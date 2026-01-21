import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo 
from std_msgs.msg import String
from cv_bridge import CvBridge 
import numpy as np
import os
import cv2
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO 
import tf2_ros
from tf2_geometry_msgs import PointStamped

# Segmentation model parameters
SEGMENTATION_CONFIDENCE_THRESHOLD = 0.4
SEGMENTATION_INPUT_SIZE = 320

# Gesture detection parameters
GESTURE_CONFIDENCE_THRESHOLD = 0.4
GESTURE_INPUT_SIZE = 640

# Mask threshold for depth extraction
MASK_THRESHOLD = 0.5

class CameraReaderSimulation(Node):
    
    def __init__(self):
        super().__init__('reader_simulation_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.input_rgb_topic = self.declare_parameter('input_rgb_topic', '/rgb_camera/image').value
        self.input_depth_topic = self.declare_parameter('input_depth_topic', '/depth_camera/image').value
        self.camera_info_topic = self.declare_parameter('input_camera_info_topic', '/depth_camera/camera_info').value

        package_share_directory = get_package_share_directory('camera_reader_simulation')

        model_path = os.path.join(package_share_directory, 'models', 'yolo11n-seg.onnx')
        self.model = YOLO(model_path, task='segment')

        gesture_package_share = get_package_share_directory('camera_reader')
        gesture_model_path = os.path.join(
            gesture_package_share,
            'models',
            'gestures',
            'yolov10n_gestures_fp32.onnx'
        )
        self.gesture_model = YOLO(gesture_model_path, task='detect')
        self.last_gesture = None

        gesture_config_path = os.path.join(package_share_directory, 'data', 'gesture_config.json')
        with open(gesture_config_path, 'r') as f:
            self.gesture_class_names = json.load(f).get('class_names', [])

        self.seg_publisher_ = self.create_publisher(Image, 'segmentation/image_raw', 2)
        self.target_publisher_ = self.create_publisher(PointStamped, 'robot/goal_point', 2)
        self.gesture_publisher_ = self.create_publisher(String, 'gesture/detected', 2)

        self.bridge = CvBridge()
        self.latest_depth_img = None
        
        self.rgb_sub = self.create_subscription(Image, self.input_rgb_topic, self.image_callback, 2)
        self.depth_sub = self.create_subscription(Image, self.input_depth_topic, self.depth_callback, 2)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 2)

        self.get_logger().info(f'Node started. Listening to {self.input_rgb_topic}')

        self.is_ready = False

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        
        self.get_logger().info(f"Paramètres reçus : fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

        self.destroy_subscription(self.camera_info_sub)
        self.get_logger().info("Subscription to CameraInfo destroyed.")

        self.is_ready = True

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"Depth error: {e}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width = frame.shape[:2]

            results = self.model(frame, verbose=False, conf=SEGMENTATION_CONFIDENCE_THRESHOLD, imgsz=SEGMENTATION_INPUT_SIZE)
            result = results[0]

            target_point = None
            display_frame = frame.copy()

            if not self.is_ready:
                self.get_logger().info("Waiting for camera info reception")

            if self.is_ready and result.masks is not None:
                display_frame = result[0].plot()

                best_box, best_mask = self.get_best_box_mask(result, height, width)

                if best_mask is not None:
                    bin_mask = (best_mask > MASK_THRESHOLD).astype(np.uint8)

                    M = cv2.moments(bin_mask)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                    else:
                        cX = int((best_box[0] + best_box[2]) / 2)
                        cY = int((best_box[1] + best_box[3]) / 2)

                    cv2.circle(display_frame, (cX, cY), 8, (0, 0, 255), -1)

                    self.process_gesture(frame, best_box)

                    z_meters = 0.0
                    if self.latest_depth_img is not None:
                        z_meters = self.get_z_meters(width, height, bin_mask)

                        x_meters = (cX - width//2) * z_meters / (self.fx * (width/640))
                        y_meters = (cY - height//2) * z_meters / (self.fy * (height/480))
                        
                        target_point = (x_meters, y_meters, z_meters)     

            self.send_segmentation(display_frame, msg)
            self.send_point_msg(target_point)

        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")
    
    def send_segmentation(self, display_frame, msg):
        seg_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
        seg_msg.header.stamp = self.get_clock().now().to_msg()
        seg_msg.header.frame_id = msg.header.frame_id
        self.seg_publisher_.publish(seg_msg)

    def process_gesture(self, full_frame, bbox):
        try:
            x1, y1, x2, y2 = map(int, bbox)
            h, w = full_frame.shape[:2]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)

            roi = full_frame[y1:y2, x1:x2]

            if roi.size > 0:
                results = self.gesture_model(roi, verbose=False, conf=GESTURE_CONFIDENCE_THRESHOLD, imgsz=GESTURE_INPUT_SIZE)
                
                if results and len(results[0].boxes) > 0:
                    best_g_box = results[0].boxes[0]
                    conf = float(best_g_box.conf[0])
                    cls_id = int(best_g_box.cls[0])

                    if cls_id < len(self.gesture_class_names):
                        gesture_name = self.gesture_class_names[cls_id]
                    else:
                        gesture_name = self.gesture_model.names[cls_id] 

                    self.get_logger().info(f"Gesture detected: {gesture_name}, Confidence: {conf:.2f}, Last: {self.last_gesture}")

                    if gesture_name != "no_gesture" and gesture_name != self.last_gesture:
                        msg = String()
                        msg.data = gesture_name
                        self.gesture_publisher_.publish(msg)
                        self.last_gesture = gesture_name

                        self.get_logger().info(f"Simulation Gesture: {gesture_name} ({conf:.2f})")

        except Exception as e:
            self.get_logger().warn(f"Error in process_gesture: {e}")   

    def send_point_msg(self, target_point):
        point_msg = PointStamped()
        point_msg.header.stamp = rclpy.time.Time().to_msg() # Avoid time problems 

        if self.is_ready and  target_point is not None:
            point_msg.header.frame_id = 'oak_d_pro_depth_optical_frame'
            point_msg.point.x = target_point[1]
            point_msg.point.y = -target_point[0]
            point_msg.point.z = target_point[2]
            try:
                point_msg = self.tf_buffer.transform(point_msg, 'map')
                point_msg.point.z = 0.0
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Waiting for transformation: {str(e)}")
                return
        else:
            return
        self.target_publisher_.publish(point_msg)

    def get_z_meters(self, width, height, bin_mask):
        try:
            if self.latest_depth_img.shape[:2] != (height, width):
                depth_resized = cv2.resize(self.latest_depth_img, (width, height), interpolation=cv2.INTER_NEAREST)
            else:
                depth_resized = self.latest_depth_img

            depth_roi = depth_resized[bin_mask == 1]
            if depth_roi.size > 0:
                valid_depths = depth_roi[depth_roi > 0]
                if valid_depths.size > 0:
                    z_meters = np.median(valid_depths)
            return z_meters

        except Exception as e:
            self.get_logger().warn(f"Depth error: {e}")

    def get_best_box_mask(self, result, height, width):
        best_score = -1
        best_mask = None
        best_box = None

        for i, box in enumerate(result.boxes):
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            
            if cls_id == 0 and conf > best_score:
                best_score = conf
                best_mask = result.masks.data[i].cpu().numpy()
                best_mask = cv2.resize(best_mask, (width, height))
                best_box = box.xyxy[0].cpu().numpy()
        
        return best_box, best_mask

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
