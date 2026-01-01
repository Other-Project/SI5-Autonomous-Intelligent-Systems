import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import numpy as np
import os
import cv2
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO 
import tf2_ros
from tf2_geometry_msgs import PointStamped, PoseStamped

class CameraReaderSimulation(Node):
    """
    Noeud ROS2 qui lit une image depuis un topic, applique YOLO Segmentation (ONNX)
    et republie les résultats.
    """
    
    def __init__(self):
        super().__init__('reader_simulation_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)



        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Nouveaux paramètres pour les topics
        self.input_rgb_topic = self.declare_parameter('input_rgb_topic', '/rgb_camera/image').value
        self.input_depth_topic = self.declare_parameter('input_depth_topic', '/depth_camera/image').value # Optionnel
        self.conf_thres = 0.4

        self.package_share_directory = get_package_share_directory('camera_reader_simulation')
        
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
        self.target_publisher_pose_ = self.create_publisher(PoseStamped, '/goal_pose', 2)
        
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
        self.fx = 554.3827056884766
        self.fy = 554.3827056884766
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
            results = self.model(frame, verbose=False, conf=self.conf_thres, imgsz=320) 
            result = results[0] # On prend le premier résultat (batch 1)

            target_point = None
            display_frame = frame.copy()

            # 3. Traitement des détections
            if result.masks is not None:
                # Dessiner les masques sur l'image pour le debug
                display_frame = result[0].plot()                
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
                        print("################ DEPTH ################################")
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
                                    z_meters = np.median(valid_depths)

                        except Exception as e:
                            self.get_logger().warn(f"Erreur calcul profondeur: {e}")
                    
                    # Cas B: Pas de profondeur, on estime une distance fixe ou arbitraire
                    print("Z meters:", z_meters)
                    if z_meters == 0.0:
                         # Valeur par défaut si pas de caméra depth (ex: 2 mètres)
                         # Ou alors on publie juste l'angle via x, y
                        z_meters = 2.0 

                    # Projection 2D -> 3D (Pinfile Model)
                    x_meters = -(cX - self.cx) * z_meters / self.fx
                    y_meters = (cY - self.cy) * z_meters / self.fy
                    
                    # Transformation vers le repère Robot (Rotation + Translation)
                    target_point = (x_meters, y_meters, z_meters)

            # 5. Publication
            now = self.get_clock().now().to_msg()
            print(msg.header.frame_id)

            # Publier l'image segmentée
            seg_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
            seg_msg.header.stamp = now
            seg_msg.header.frame_id = msg.header.frame_id
            self.seg_publisher_.publish(seg_msg)

            # Publier le point cible
            point_msg = PointStamped()
            point_msg.header.stamp = now
            point_msg.header.frame_id = 'oak_d_pro_depth_optical_frame' # Ou le frame de votre robot
            
            if target_point is not None:
                point_msg.point.x = target_point[0]
                point_msg.point.y = target_point[1]
                point_msg.point.z = target_point[2]
                print("not________________none")
            else:
                point_msg.point.x = 0.0
                point_msg.point.y = 0.0
                point_msg.point.z = 0.0


            try:
                point_robot = self.tf_buffer.transform(point_msg, 'base_link')
                point_robot.point.z = 0.0
                self.convert_point_to_pose(point_robot)
                # C. Publication du message transformé
                self.target_publisher_.publish(point_robot)   
                self.get_logger().info(f"Point publié dans base_link: {point_robot.point.x}, {point_robot.point.y}, {point_robot.point.z}")  
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        self.get_logger().warn(f"Attente de la transformation : {str(e)}")

        except Exception as e:
            self.get_logger().error(f"Erreur dans image_callback: {e}")

    def convert_point_to_pose(self,point_msg):
        pose_msg = PoseStamped()
        
        # 1. On copie le header (important pour le timestamp et le frame_id 'map')
        pose_msg.header = point_msg.header
        
        # 2. On copie les coordonnées de position
        pose_msg.pose.position.x = point_msg.point.x
        pose_msg.pose.position.y = point_msg.point.y
        pose_msg.pose.position.z = point_msg.point.z
        
        # 3. On définit une orientation par défaut
        # Si on ne met rien, le robot risque de refuser le but car le quaternion (0,0,0,0) est invalide.
        # On utilise souvent (0,0,0,1) pour dire "pas de rotation particulière".
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        pose_msg.header.stamp = rclpy.time.Time().to_msg()
        pose_msg.header.frame_id = "base_link"

        pose_sur_la_map = self.tf_buffer.transform(pose_msg, "map", timeout=rclpy.duration.Duration(seconds=1))

        self.target_publisher_pose_.publish(pose_sur_la_map)
        
        return pose_msg

    def destroy_node(self):
        # Nettoyage
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraReaderSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:    
        rclpy.shutdown()

if __name__ == '__main__':
    main()