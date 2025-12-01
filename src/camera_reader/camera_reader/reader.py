import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import math
import json
import depthai as dai
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from .yolo_api import Segment 

class CameraReader(Node):
    def __init__(self):
        super().__init__('camera_reader_node')

        self.get_logger().info('CameraReader node started and initializing DepthAI...')

        self.package_share_directory = get_package_share_directory('camera_reader')


        try:
            config_path = os.path.join(self.package_share_directory, 'data', 'config.json')
            with open(config_path, "r") as config:
                self.model_data = json.load(config)
                pass
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement de la configuration: {e}")

        self.preview_img_width = self.model_data["input_width"]
        self.preview_img_height = self.model_data["input_height"]
        self.input_shape = [1, 3, self.preview_img_height, self.preview_img_width]
        
        try:
            blob_filename = "yolo11n-seg640x640.blob"
            self.path_to_yolo_blob = os.path.join(self.package_share_directory, 'models', blob_filename)
            
            self.get_logger().info(f"Chemin du Blob défini à: {self.path_to_yolo_blob}")
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la détermination du chemin du Blob : {e}")
            raise

        # --- 2. INIT DEPTHAI PIPELINE AND CLASSES ---
        self._init_depthai_pipeline()
        
        # 3. INIT DE Segment CLASSE
        self.yoloseg = Segment(
            input_shape=self.input_shape,
            input_height=self.preview_img_height,
            input_width=self.preview_img_width,
            conf_thres=0.1,
            iou_thres=0.5,
            num_masks=32
        )
        self.yoloseg.prepare_input_for_oakd((self.preview_img_height, self.preview_img_width))

        # --- 4. INIT ROS 2 PUBLISHER ---
        self.seg_publisher_ = self.create_publisher(Image, 'segmentation/image_raw', 10)
        self.image_publisher_ = self.create_publisher(Image, 'segmentation/through/image_raw', 10)
        self.bridge = CvBridge()
        self.timer_period = 0.033 
        self.timer = self.create_timer(self.timer_period, self._timer_callback)
        self.get_logger().info('DepthAI pipeline launched. Ready to publish segmentation results.')

    def _init_depthai_pipeline(self):

        self.device = dai.Device()
        self.pipeline = dai.Pipeline()

        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_rgb.setPreviewSize(self.preview_img_width, self.preview_img_height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        nn = self.pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(self.path_to_yolo_blob)
        cam_rgb.preview.link(nn.input)

        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb_stream")
        cam_rgb.preview.link(xout_rgb.input) 

        xout_nn = self.pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn_results")
        
        nn.out.link(xout_nn.input)

        self.device.startPipeline(self.pipeline)

        self.q_rgb = self.device.getOutputQueue(name="rgb_stream", maxSize=4, blocking=False)
        self.q_yolo = self.device.getOutputQueue(name="nn_results", maxSize=4, blocking=False)
        
        self.get_logger().info('DepthAI device and pipeline configured.')


    def _timer_callback(self):
        try:
            in_rgb = self.q_rgb.tryGet()
            in_nn = self.q_yolo.tryGet()

            if in_rgb is not None and in_nn is not None:
                frame = in_rgb.getCvFrame()

                ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
                ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                self.image_publisher_.publish(ros_image_msg)
                
                layer0 = np.array(in_nn.getLayerFp16("output0")).reshape(self.model_data["shapes"]["output0"])
                layer1 =np.array( in_nn.getLayerFp16("output1")).reshape(self.model_data["shapes"]["output1"])

                self.yoloseg.segment_objects_from_oakd(layer0, layer1)

                display_frame = self.yoloseg.draw_masks(frame, draw_scores=True, mask_alpha=0.5)

                ros_image_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="rgb8")
                ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                self.seg_publisher_.publish(ros_image_msg)

        except Exception as e:
            self.get_logger().error(f"Erreur traitement dans le callback: {e}")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraReader()
    try:
        rclpy.spin(node)
    finally:    
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()