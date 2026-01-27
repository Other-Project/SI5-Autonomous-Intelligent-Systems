import tf2_geometry_msgs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge 
import json
import depthai as dai
import numpy as np
import os
import cv2
import threading 
from ament_index_python.packages import get_package_share_directory
import std_msgs
from .yolo_api import Segment 
import tf2_ros
import tf2_geometry_msgs # Required for transforming PointStamped
import onnxruntime as ort

# Camera settings
CAMERA_ANGLE = 45.0 # degrees
CAMERA_X = 0.0
CAMERA_Y = 0.14
CAMERA_Z = -0.10
CAMERA_FPS = 30

# Segmentation parameters
SEGMENTATION_CONFIDENCE_THRESHOLD = 0.4
SEGMENTATION_IOU_THRESHOLD = 0.5
SEGMENTATION_NUM_MASKS = 32

# Mask threshold for depth extraction
MASK_THRESHOLD = 0.5

# Gesture detection parameters
GESTURE_CONFIDENCE_THRESHOLD = 0.8
GESTURE_MODEL_INPUT_SIZE = (640, 640)  # Input size for gesture model

class CameraReader(Node):
    """ROS2 node for OAK-D camera reading and YOLO segmentation.
    
    This node handles RGB and depth image capture from a DepthAI camera, performs object segmentation with YOLO and publishes the results.
    """
    
    def __init__(self):
        """Initialize the CameraReader node.
        
        Configures the DepthAI camera, loads the YOLO model, initializes ROS2 publishers and starts the image capture thread.
        """
        super().__init__('camera_reader_node')
        self.package_share_directory = get_package_share_directory('camera_reader')

        config_path = os.path.join(self.package_share_directory, 'data', 'config.json')
        with open(config_path, "r") as config:
            self.model_data = json.load(config)

        # Set image dimensions and input shape
        self.preview_img_width = self.model_data["input_width"]
        self.preview_img_height = self.model_data["input_height"]
        self.input_shape = [1, 3, self.preview_img_height, self.preview_img_width]
        
        blob_filename = "yolo11n-seg_384x640_shave_8.blob"
        self.path_to_yolo_blob = os.path.join(self.package_share_directory, 'models/segmentation', blob_filename)

        self._init_depthai_pipeline()
        
        # Initialize YOLO segmentation
        self.yoloseg = Segment(
            input_shape=self.input_shape,
            input_height=self.preview_img_height,
            input_width=self.preview_img_width,
            class_names=self.model_data["class_names"],
            conf_thres=SEGMENTATION_CONFIDENCE_THRESHOLD,
            iou_thres=SEGMENTATION_IOU_THRESHOLD,
            num_masks=SEGMENTATION_NUM_MASKS
        )
        self.yoloseg.prepare_input_for_oakd((self.preview_img_height, self.preview_img_width))

        # Initialize gesture detection model
        self._init_gesture_model()
        self.last_gesture = None
        
        # Initialize publishers
        self.seg_publisher_ = self.create_publisher(Image, 'segmentation/image_raw', 2)
        self.image_publisher_ = self.create_publisher(Image, 'segmentation/through/image_raw', 2)
        self.target_publisher_ = self.create_publisher(PointStamped, 'robot/goal_point', 2)
        self.pcl_publisher_ = self.create_publisher(PointCloud2, 'camera/pointcloud', 2)
        self.gesture_publisher_ = self.create_publisher(String, 'gesture/detected', 2)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        
        # Initialize threading for camera loop
        self.running = True
        self.thread = threading.Thread(target=self._run_camera_loop)
        self.thread.start()
        
        self.get_logger().info('CameraReader started with Threading.')

    def _init_gesture_model(self):
        """Initialize the gesture detection model.
        
        Loads the ONNX gesture detection model.
        The model is applied to the detected person bounding boxes.
        """
        gesture_model_path = os.path.join(self.package_share_directory, 'models/gestures', 'YOLOv10n_gestures_640_INT8.onnx')
        
        self.gesture_session = ort.InferenceSession(
            gesture_model_path,
            providers=['CPUExecutionProvider']
        )
        self.gesture_input_name = self.gesture_session.get_inputs()[0].name
        self.gesture_output_names = [output.name for output in self.gesture_session.get_outputs()]
        self.last_gesture = None
        
        # Load gesture class names if config exists
        gesture_config_path = os.path.join(self.package_share_directory, 'data', 'gesture_config.json')
        with open(gesture_config_path, 'r') as f:
            gesture_config = json.load(f)
            self.gesture_class_names = gesture_config.get('class_names', [])
            
    def _init_depthai_pipeline(self):
        """Initialize the DepthAI pipeline for the OAK-D camera.
        
        Configures RGB and stereo cameras, the neural network for YOLO and establishes connections between different pipeline nodes.
        Push the pipeline on the camera.
        """
        self.device = dai.Device()
        self.pipeline = dai.Pipeline()

        # Color camera node
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(self.preview_img_width, self.preview_img_height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(CAMERA_FPS)

        # Mono cameras for depth
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)

        # Stereo depth node
        stereo = self.pipeline.create(dai.node.StereoDepth)

        # Set resolutions and board sockets for mono cameras
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Configure stereo depth
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        # Link mono cameras to stereo depth
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        # Image manip for depth resizing
        manip_depth = self.pipeline.create(dai.node.ImageManip)
        manip_depth.initialConfig.setResize(self.preview_img_width, self.preview_img_height)
        manip_depth.initialConfig.setFrameType(dai.ImgFrame.Type.RAW16)
        stereo.depth.link(manip_depth.inputImage)

        # Point cloud 
        pointcloud = self.pipeline.create(dai.node.PointCloud)
        pointcloud.initialConfig.setSparse(False)
        stereo.depth.link(pointcloud.inputDepth)

        # Neural network node
        nn = self.pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(self.path_to_yolo_blob)
        cam_rgb.preview.link(nn.input)

        # NN output
        xout_nn = self.pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn_results")
        nn.out.link(xout_nn.input)

        # RGB passthrough output
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb_pass")
        nn.passthrough.link(xout_rgb.input) 

        # Depth output
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        manip_depth.out.link(xout_depth.input)

        # Point cloud output
        xout_pcl = self.pipeline.create(dai.node.XLinkOut)
        xout_pcl.setStreamName("pcl")
        pointcloud.outputPointCloud.link(xout_pcl.input)

        # Start the pipeline
        self.device.startPipeline(self.pipeline)

        # Get output queues
        self.q_rgb = self.device.getOutputQueue(name="rgb_pass", maxSize=2, blocking=False)
        self.q_nn = self.device.getOutputQueue(name="nn_results", maxSize=2, blocking=False)
        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=2, blocking=False)
        self.q_pcl = self.device.getOutputQueue(name="pcl", maxSize=2, blocking=False)

        ## Rotation matrix for camera to robot frame transformation
        theta = np.radians(180.0 - CAMERA_ANGLE)
        c, s = np.cos(theta), np.sin(theta)
        self.rotation_matrix = np.array([
            [1, 0, 0],
            [0, c, -s],
            [0, s, c]
        ])

        # Get camera intrinsics
        calibData = self.device.readCalibration()
        self.intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB, self.preview_img_width, self.preview_img_height)

    def _transform_camera_to_robot(self, points_np):
        """Transform points from camera frame to robot frame.

        Args:
            points_np (np.array): Nx3 array of points in camera
        Returns:
            np.array: Nx3 array of points in robot frame
        """
        points_rotated = points_np @ self.rotation_matrix.T

        points_rotated[:, 0] += CAMERA_X
        points_rotated[:, 1] += CAMERA_Y
        points_rotated[:, 2] += CAMERA_Z
        
        return points_rotated
    
    def _compute_3d_point_from_depth(self, cX, cY, valid_depths):
        """Calculate 3D coordinates from 2D centroid and depth values.

        Args:
            cX (int): X coordinate of the centroid in the image.
            cY (int): Y coordinate of the centroid in the image.
            valid_depths (np.array): Array of valid depth values within the mask.
        Returns:
            np.array: 3D coordinates (X, Y, Z) in meters.
        """
        # Compute average depth
        avg_depth = np.median(valid_depths)
        
        # Focal lengths
        fx = self.intrinsics[0][0]
        fy = self.intrinsics[1][1]

        # Optical centers
        cx_int = self.intrinsics[0][2]
        cy_int = self.intrinsics[1][2]

        # Convert to real-world coordinates
        z_meters = avg_depth / 1000.0
        x_meters = (cX - cx_int) * z_meters / fx
        y_meters = -(cY - cy_int) * z_meters / fy

        return np.array([[x_meters, y_meters, z_meters]])
    
    def _compute_mask_centroid(self, person_mask, target_idx):
        """Calculate the centroid of a segmentation mask.
        
        Args:
            person_mask (np.array): Binary mask of the detected object
            target_idx (int): Index of the target in the YOLO results
            
        Returns:
            tuple: (cX, cY) coordinates of the centroid
        """
        M = cv2.moments(person_mask)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            # Fallback to bounding box center if mask is empty
            box = self.yoloseg.boxes[target_idx]
            cX = int((box[0] + box[2]) / 2)
            cY = int((box[1] + box[3]) / 2)
            self.get_logger().warn("Empty mask, using Bounding Box center")
        
        return cX, cY
    
    def _apply_letterbox(self, image, target_size, color=(114, 114, 114)):
        """Resize image to the target size while preserving aspect ratio using padding.
        
        Args:
            image (np.array): Image to resize.
            target_size (tuple): Desired output size (width, height).
            color (tuple): BGR color for the padding.
            
        Returns:
            np.array: Letterboxed image.
        """
        h_orig, w_orig = image.shape[:2]
        tw, th = target_size
        
        # Scale factor (ratio)
        r = min(tw / w_orig, th / h_orig)
        
        # Compute resizing dimensions
        new_w, new_h = int(round(w_orig * r)), int(round(h_orig * r))
        
        # Resize proportionally
        if (w_orig, h_orig) != (new_w, new_h):
            image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        
        # Compute padding
        dw = (tw - new_w) / 2
        dh = (th - new_h) / 2
        
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        
        # Add borders
        return cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)

    def _detect_gesture(self, frame, bbox):
        """Detect gesture within a person's bounding box.
        
        Args:
            frame (np.array): Full frame from camera.
            bbox (np.array): Bounding box [x1, y1, x2, y2] of the person.
            
        Returns:
            tuple: (gesture_name, confidence) or (None, 0.0) if no gesture detected.
        """
        if self.gesture_session is None:
            return None, 0.0
        
        try:
            # Extract and clip ROI from bounding box
            x1, y1, x2, y2 = map(int, bbox)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
            
            roi = frame[y1:y2, x1:x2]
            if roi.size == 0:
                return None, 0.0
            
            # Apply letterboxing to preserve aspect ratio
            roi_letterboxed = self._apply_letterbox(roi, GESTURE_MODEL_INPUT_SIZE)
            
            # Preprocess for ONNX inference
            roi_rgb = cv2.cvtColor(roi_letterboxed, cv2.COLOR_BGR2RGB)
            roi_normalized = roi_rgb.astype(np.float32) / 255.0
            roi_transposed = np.transpose(roi_normalized, (2, 0, 1))
            roi_input = np.expand_dims(roi_transposed, axis=0)
            
            # Run inference
            outputs = self.gesture_session.run(
                self.gesture_output_names,
                {self.gesture_input_name: roi_input}
            )
            
            predictions = outputs[0][0]
            
            if len(predictions.shape) == 2:
                # Get best class and confidence
                scores = predictions[:, 4]
                best_idx = scores.argmax()
                confidence = float(scores[best_idx])
                
                if confidence >= GESTURE_CONFIDENCE_THRESHOLD:
                    class_id = int(predictions[best_idx, 5])
                    gesture_name = (self.gesture_class_names[class_id] 
                                   if class_id < len(self.gesture_class_names) 
                                   else f"gesture_{class_id}")

                    if gesture_name == "no_gesture" or gesture_name == self.last_gesture:
                        return None, 0.0

                    self.last_gesture = gesture_name
                    return gesture_name, confidence
            
            return None, 0.0
            
        except Exception as e:
            self.get_logger().warn(f'Gesture detection error: {e}')
            return None, 0.0
    
    def _publish_goal_point(self, point, now):
        """Publish the target goal point as a PoseStamped message.

        Args:
            point (list): 3D coordinates [X, Y, Z] in meters.
            now (rclpy.time.Time): Current ROS2 time for the message header.            
        """
        point_msg = PointStamped()
        point_msg.header.stamp = now
        point_msg.header.frame_id = "base_link"

        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = point[2]
        try:
            point_msg = self.tf_buffer.transform(point_msg, 'map')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Waiting for transformation: {str(e)}")
            return

        self.target_publisher_.publish(point_msg)

    def _run_camera_loop(self):
        """Main loop for image capture and processing.
        
        Retrieves RGB and depth frames from the camera, runs YOLO segmentation, computes 3D coordinates of detected objects and publishes results to ROS2 topics.
        """
        while self.running and rclpy.ok():
            try:
                # Get frames from DepthAI queues
                in_rgb = self.q_rgb.get() 
                in_nn = self.q_nn.get()
                in_depth = self.q_depth.get() 
                in_pcl = self.q_pcl.tryGet()

                # Process frames
                frame = in_rgb.getCvFrame()
                depth_frame = in_depth.getFrame()
                now = self.get_clock().now().to_msg()

                if (in_pcl is not None and
                    self.pcl_publisher_.get_subscription_count() > 0):

                    points = in_pcl.getPoints().reshape(-1, 3) / 1000.0

                    # Transform points to robot frame
                    points_robot = self._transform_camera_to_robot(points)

                    header = std_msgs.msg.Header()
                    header.stamp = now
                    header.frame_id = "base_link"

                    pc2_msg = pc2.create_cloud_xyz32(header, points_robot)
                    self.pcl_publisher_.publish(pc2_msg)

                # Get NN outputs
                layer0 = np.array(in_nn.getLayerFp16("output0")).reshape(self.model_data["shapes"]["output0"])
                layer1 = np.array(in_nn.getLayerFp16("output1")).reshape(self.model_data["shapes"]["output1"])

                # Perform segmentation
                self.yoloseg.segment_objects_from_oakd(layer0, layer1)

                if len(self.yoloseg.class_ids) > 0:
                    # Find the person with the highest score
                    target_idx = np.argmax(self.yoloseg.scores)
                    person_mask = self.yoloseg.mask_maps[target_idx]
                    person_bbox = self.yoloseg.boxes[target_idx]
                    
                    # Calculate centroid of the mask
                    cX, cY = self._compute_mask_centroid(person_mask, target_idx)

                    # Extract depth values within the mask
                    depth_roi = depth_frame[person_mask > MASK_THRESHOLD] 
                    valid_depths = depth_roi[depth_roi > 0]

                    if valid_depths.size > 0:
                        # Compute 3D coordinates from depth
                        point_cam_np = self._compute_3d_point_from_depth(cX, cY, valid_depths)

                        # Transform to robot frame
                        point_robot = self._transform_camera_to_robot(point_cam_np)[0]

                        self.get_logger().info(f"Goal point : ({point_robot[0]:.2f}, {point_robot[1]:.2f}, {point_robot[2]:.2f})")
                        #set Z axis to 0
                        point_robot[2] = 0.0
                        self.get_logger().info(f"Adjusted goal point : ({point_robot[0]:.2f}, {point_robot[1]:.2f}, {point_robot[2]:.2f})")

                        # Publish the target point
                        self._publish_goal_point(point_robot, now)

                        # Visualize the centroid on the frame
                        cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)
                    
                    # Gesture detection on person bounding box
                    gesture_name, gesture_conf = self._detect_gesture(frame, person_bbox)
                    if gesture_name is not None:
                        gesture_msg = String()
                        gesture_msg.data = gesture_name
                        self.gesture_publisher_.publish(gesture_msg)
                        self.get_logger().debug(f'Detected gesture: {gesture_name} ({gesture_conf:.2f})')
                
                else:
                    # Publish the target point
                    self._publish_goal_point([0.0, 0.0, 0.0], now)
            
                # Publish images
                if self.image_publisher_.get_subscription_count() > 0:
                    ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    ros_image_msg.header.stamp = now
                    self.image_publisher_.publish(ros_image_msg)

                # Draw segmentation masks on frame
                if self.seg_publisher_.get_subscription_count() > 0:
                    display_frame = self.yoloseg.draw_masks(frame, draw_scores=True, mask_alpha=0.5)
                    display_frame = cv2.resize(display_frame, (0, 0), fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
                    ros_seg_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
                    ros_seg_msg.header.stamp = now
                    self.seg_publisher_.publish(ros_seg_msg)

            except RuntimeError:
                break
            except Exception as e:
                self.get_logger().error(f"Error in thread: {e}")

    def destroy_node(self):
        """Gracefully stop the node and release resources.
        
        Stops the image capture thread and waits for its termination before destroying the ROS2 node.
        """
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        super().destroy_node()

def main(args=None):
    """Main entry point for the CameraReader node.
    
    Args:
        args: Command line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    node = CameraReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:    
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
