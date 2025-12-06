import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge 
import json
import depthai as dai
import numpy as np
import os
import cv2
import threading 
from ament_index_python.packages import get_package_share_directory
from .yolo_api import Segment 

class CameraReader(Node):
    def __init__(self):
        super().__init__('camera_reader_node')

        self.get_logger().info('CameraReader node started...')
        self.package_share_directory = get_package_share_directory('camera_reader')

        # --- Load Config ---
        try:
            config_path = os.path.join(self.package_share_directory, 'data', 'config.json')
            with open(config_path, "r") as config:
                self.model_data = json.load(config)
        except Exception as e:
            self.get_logger().error(f"Erreur config: {e}")
            return

        self.preview_img_width = self.model_data["input_width"]
        self.preview_img_height = self.model_data["input_height"]
        self.input_shape = [1, 3, self.preview_img_height, self.preview_img_width]
        
        # --- Load Blob ---
        try:
            blob_filename = "yolo11n-seg640x640.blob"
            self.path_to_yolo_blob = os.path.join(self.package_share_directory, 'models', blob_filename)
        except Exception as e:
            self.get_logger().error(f"Erreur Blob path: {e}")
            raise

        # --- Init Pipeline ---
        self._init_depthai_pipeline()
        
        # --- Init Segment ---
        self.yoloseg = Segment(
            input_shape=self.input_shape,
            input_height=self.preview_img_height,
            input_width=self.preview_img_width,
            conf_thres=0.1,
            iou_thres=0.5,
            num_masks=32
        )
        self.yoloseg.prepare_input_for_oakd((self.preview_img_height, self.preview_img_width))

        # --- Init ROS ---
        self.seg_publisher_ = self.create_publisher(Image, 'segmentation/image_raw', 2)
        self.image_publisher_ = self.create_publisher(Image, 'segmentation/through/image_raw', 2)
        self.seg_compressed_publisher_ = self.create_publisher(CompressedImage, 'segmentation/image_raw/compressed', 2)
        self.image_compressed_publisher_ = self.create_publisher(CompressedImage, 'segmentation/through/image_raw/compressed', 2)
        self.bridge = CvBridge()
        
        self.running = True
        self.thread = threading.Thread(target=self._run_camera_loop)
        self.thread.start()
        
        self.get_logger().info('DepthAI thread started. Ready.')

    def _init_depthai_pipeline(self):
        self.device = dai.Device()
        self.pipeline = dai.Pipeline()

        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_rgb.setPreviewSize(self.preview_img_width, self.preview_img_height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        nn = self.pipeline.create(dai.node.NeuralNetwork)
        nn.setBlobPath(self.path_to_yolo_blob)
        nn.input.setBlocking(False) 
        
        cam_rgb.preview.link(nn.input)

        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb_stream")
        nn.passthrough.link(xout_rgb.input) 

        xout_nn = self.pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName("nn_results")
        nn.out.link(xout_nn.input)

        self.device.startPipeline(self.pipeline)

        self.q_rgb = self.device.getOutputQueue(name="rgb_stream", maxSize=2, blocking=False)
        self.q_yolo = self.device.getOutputQueue(name="nn_results", maxSize=2, blocking=False)

    def _run_camera_loop(self):
        """Ce code tourne en parallèle dans un thread séparé"""
        while self.running and rclpy.ok():
            try:
                in_nn = self.q_yolo.get() 
                in_rgb = self.q_rgb.get() 

                if in_rgb is not None and in_nn is not None:
                    frame = in_rgb.getCvFrame()
                    now = self.get_clock().now().to_msg()

                    ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    ros_image_msg.header.stamp = now
                    self.image_publisher_.publish(ros_image_msg)

                    ros_image_msg_compressed = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
                    ros_image_msg_compressed.header.stamp = now
                    self.image_compressed_publisher_.publish(ros_image_msg_compressed)

                    layer0 = np.array(in_nn.getLayerFp16("output0")).reshape(self.model_data["shapes"]["output0"])
                    layer1 = np.array(in_nn.getLayerFp16("output1")).reshape(self.model_data["shapes"]["output1"])

                    self.yoloseg.segment_objects_from_oakd(layer0, layer1)

                    display_frame = self.yoloseg.draw_masks(frame, draw_scores=True, mask_alpha=0.5)

                    seg_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
                    seg_msg.header.stamp = now
                    self.seg_publisher_.publish(seg_msg)

                    seg_msg_compressed = self.bridge.cv2_to_compressed_imgmsg(display_frame, dst_format='jpg')
                    seg_msg_compressed.header.stamp = now
                    self.seg_compressed_publisher_.publish(seg_msg_compressed)

            except Exception as e:
                self.get_logger().error(f"Erreur thread caméra: {e}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        super().destroy_node()

def main(args=None):
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