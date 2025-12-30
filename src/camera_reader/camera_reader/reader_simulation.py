import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge 
import numpy as np
import os
import cv2
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO 

class CameraReaderSimulation(Node):
    """
    Noeud ROS2 qui lit une image depuis un topic, applique YOLO Segmentation (ONNX)
    et republie les résultats.
    """
    
    def __init__(self):
        super().__init__('onnx_camera_reader', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # --- Paramètres ---
        self.camera_angle = self.get_parameter('camera_angle').get_parameter_value().double_value
        self.camera_x = self.get_parameter('camera_x').get_parameter_value().double_value
        self.camera_y = self.get_parameter('camera_y').get_parameter_value().double_value
        self.camera_z = self.get_parameter('camera_z').get_parameter_value().double_value
        
        # Nouveaux paramètres pour les topics
        self.input_rgb_topic = self.declare_parameter('input_rgb_topic', '/camera/image_raw').value
        self.input_depth_topic = self.declare_parameter('input_depth_topic', '/camera/depth_raw').value # Optionnel
        self.conf_thres = 0.4

        self.package_share_directory = get_package_share_directory('camera_reader')
        
        # --- Chargement du modèle ONNX ---
        # Note: On utilise Ultralytics pour charger l'ONNX. C'est beaucoup plus robuste 
        # pour la segmentation que de faire du post-traitement matriciel manuel.
        model_filename = "yolo11n-seg.onnx" 
        path_to_model = os.path.join(self.package_share_directory, 'models', model_filename)
        
        self.get_logger().info(f"Chargement du modèle ONNX : {path_to_model}")
        try:
            self.model = YOLO(path_to_model, task='segment')
        except Exception as e:
            self.get_logger().error(f"Impossible de charger le modèle : {e}")
            raise e

        # --- Publishers ---
        self.seg_publisher_ = self.create_publisher(Image, 'segmentation/image_raw', 2)
        self.target_publisher_ = self.create_publisher(PointStamped, 'robot/goal_point', 2)
        
        # --- Subscribers ---
        self.bridge = CvBridge()
        
        # Subscription à l'image couleur
        self.rgb_sub = self.create_subscription(
            Image, 
            self.input_rgb_topic, 
            self.image_callback, 
            2
        )
        
        # Subscription à la profondeur (Optionnel mais nécessaire pour le calcul X,Y,Z réel)
        self.latest_depth_img = None
        self.depth_sub = self.create_subscription(
            Image,
            self.input_depth_topic,
            self.depth_callback,
            2
        )


        ### TODO à voir 
        # Intrinsèques Caméra (A ajuster selon votre Webcam/Raspberry Cam !)
        # Valeurs par défaut approximatives pour une image 640x480
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0

        self.get_logger().info(f'OnnxCameraReader démarré. Ecoute sur {self.input_rgb_topic}')

    def depth_callback(self, msg):
        """Callback simple pour garder en mémoire la dernière frame de profondeur."""
        try:
            # Attention à l'encoding (souvent 16UC1 pour la profondeur en mm)
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"Erreur depth callback: {e}")

    def image_callback(self, msg):
        """Callback principal : Reçoit l'image, infère, publie."""
        try:
            # 1. Conversion ROS -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width = frame.shape[:2]
            
            # Mise à jour des intrinsèques si la taille change (simple approximation)
            self.cx = width / 2.0
            self.cy = height / 2.0

            # 2. Inférence ONNX via Ultralytics
            # imgsz définit la taille d'entrée du modèle
            results = self.model(frame, verbose=False, conf=self.conf_thres, imgsz=640) 
            result = results[0] # On prend le premier résultat (batch 1)

            target_point = None
            display_frame = frame.copy()

            # 3. Traitement des détections
            if result.masks is not None:
                # Dessiner les masques sur l'image pour le debug
                display_frame = result.plot(img=frame.copy(), alpha=0.5)
                
                # Trouver la personne avec le score le plus élevé
                # Classes: 0 est souvent 'person' dans COCO
                best_score = -1
                best_mask = None
                best_box = None

                for i, box in enumerate(result.boxes):
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Filtrer pour ne garder que la classe 'person' (ID 0 pour COCO)
                    # Si vous utilisez un modèle custom, ajustez l'ID.
                    if cls_id == 0 and conf > best_score:
                        best_score = conf
                        # Récupérer le masque binaire redimensionné à la taille de l'image originale
                        best_mask = result.masks.data[i].cpu().numpy()
                        # Le masque sort souvent en float 640x640, il faut le redimensionner à l'image
                        best_mask = cv2.resize(best_mask, (width, height))
                        best_box = box.xyxy[0].cpu().numpy() # x1, y1, x2, y2

                if best_mask is not None:
                    # Binarisation du masque (0 ou 1)
                    bin_mask = (best_mask > 0.5).astype(np.uint8)

                    # Calcul du centroïde
                    M = cv2.moments(bin_mask)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        # Fallback sur le centre de la boite
                        cX = int((best_box[0] + best_box[2]) / 2)
                        cY = int((best_box[1] + best_box[3]) / 2)

                    # Dessin du point cible
                    cv2.circle(display_frame, (cX, cY), 8, (0, 0, 255), -1)

                    # 4. Calcul de la position 3D
                    z_meters = 0.0
                    
                    # Cas A: On a une image de profondeur
                    if self.latest_depth_img is not None:
                        try:
                            # Redimensionner la profondeur si nécessaire pour matcher RGB
                            if self.latest_depth_img.shape[:2] != (height, width):
                                depth_resized = cv2.resize(self.latest_depth_img, (width, height), interpolation=cv2.INTER_NEAREST)
                            else:
                                depth_resized = self.latest_depth_img

                            # On prend la profondeur au point cX, cY
                            # Ou mieux : la médiane de la zone masquée
                            depth_roi = depth_resized[bin_mask == 1]
                            if depth_roi.size > 0:
                                valid_depths = depth_roi[depth_roi > 0]
                                if valid_depths.size > 0:
                                    # Profondeur en mm convertie en mètres
                                    z_meters = np.median(valid_depths) / 1000.0
                        except Exception as e:
                            self.get_logger().warn(f"Erreur calcul profondeur: {e}")
                    
                    # Cas B: Pas de profondeur, on estime une distance fixe ou arbitraire
                    if z_meters == 0.0:
                         # Valeur par défaut si pas de caméra depth (ex: 2 mètres)
                         # Ou alors on publie juste l'angle via x, y
                        z_meters = 2.0 

                    # Projection 2D -> 3D (Pinfile Model)
                    x_meters = (cX - self.cx) * z_meters / self.fx
                    y_meters = (cY - self.cy) * z_meters / self.fy
                    
                    # Transformation vers le repère Robot (Rotation + Translation)
                    target_point = self.transform_camera_to_robot(x_meters, y_meters, z_meters)

            # 5. Publication
            now = self.get_clock().now().to_msg()
            
            # Publier l'image segmentée
            seg_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
            seg_msg.header.stamp = now
            seg_msg.header.frame_id = msg.header.frame_id
            self.seg_publisher_.publish(seg_msg)

            # Publier le point cible
            point_msg = PointStamped()
            point_msg.header.stamp = now
            point_msg.header.frame_id = "base_link" # Ou le frame de votre robot
            
            if target_point is not None:
                point_msg.point.x = target_point[0]
                point_msg.point.y = target_point[1]
                point_msg.point.z = target_point[2]
            else:
                point_msg.point.x = 0.0
                point_msg.point.y = 0.0
                point_msg.point.z = 0.0
                
            self.target_publisher_.publish(point_msg)

        except Exception as e:
            self.get_logger().error(f"Erreur dans image_callback: {e}")

    def transform_camera_to_robot(self, x_cam, y_cam, z_cam):
        """Applique la rotation et translation définies dans les paramètres."""
        # Rotation autour de X (pitch) basée sur camera_angle
        # Note: Dans OpenCV Z est l'avant, X la droite, Y le bas.
        # Sur un robot, souvent X est l'avant. Ajustez selon votre TF tree.
        
        theta = np.radians(180.0 - self.camera_angle)
        point_cam = np.array([x_cam, y_cam, z_cam])

        # Matrice de rotation (adaptée de votre code original)
        c, s = np.cos(theta), np.sin(theta)
        R_x = np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])

        point_rot = np.dot(R_x, point_cam)

        x_rob = point_rot[0] + self.camera_x
        y_rob = point_rot[1] + self.camera_y
        z_rob = point_rot[2] + self.camera_z
        
        return (x_rob, y_rob, z_rob)

    def destroy_node(self):
        # Nettoyage
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OnnxCameraReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:    
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()