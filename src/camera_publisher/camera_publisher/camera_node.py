import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
from ament_index_python.packages import get_package_share_directory
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.br = CvBridge()

        # Load calibration file
        pkg_dir = get_package_share_directory("camera_publisher")
        calib_path = os.path.join(pkg_dir, "config", "camera_calib_params.yaml")
        self.camera_info_msg = self.load_camera_info(calib_path)

        # GStreamer pipeline for IMX219 on Jetson
        gst_str = (
            "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink"
        )

        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Failed to open IMX219 via GStreamer pipeline")
            exit(1)

        self.get_logger().info("✅ IMX219 camera opened successfully")
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def load_camera_info(self, path):
        with open(path, 'r') as f:
            calib = yaml.safe_load(f)

        cam_info = CameraInfo()
        cam_info.width = calib["image_width"]
        cam_info.height = calib["image_height"]
        cam_info.distortion_model = calib["distortion_model"]
        cam_info.d = calib["distortion_coefficients"]["data"]
        cam_info.k = calib["camera_matrix"]["data"]
        cam_info.r = calib["rectification_matrix"]["data"]
        cam_info.p = calib["projection_matrix"]["data"]
        return cam_info

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Failed to grab frame.")
            return

        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        self.camera_info_msg.header = msg.header

        self.image_pub.publish(msg)
        self.info_pub.publish(self.camera_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
