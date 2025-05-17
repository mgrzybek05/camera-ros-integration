import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('camera_id', 0)
        self.camera_id = self.get_parameter('camera_id').value

        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)  # 1.0/ 15.0 = 15 FPS

        self.cap = cv2.VideoCapture(self.camera_id) #/dev/video*, where * is camera_id
        if not self.cap.isOpened():
            self.get_logger().error(f"Camera {self.camera_id} could not be opened.")

    # Every 15th of a second, take a picture and publish it to the image_raw topic
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Failed to read from camera")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()