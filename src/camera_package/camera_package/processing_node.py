import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing_node')
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'image_processed', 10)
        self.marker_pub = self.create_publisher(String, 'aruco_detections', 10) # aruco markers as list
        self.color_pub = self.create_publisher(String, 'color_detections', 10) # colors as list
        self.bridge = CvBridge()

        # Load Aruco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100) # CIRC 2025 rules state 4x4 100 is being used
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def listener_callback(self, msg):

        # ArUco marker detection
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        # Color detection and output
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert to rgb

        # red range
        lower_red = (0, 120, 70)
        upper_red = (10, 255, 255)

        # blue range
        lower_blue = (100, 150, 0)
        upper_blue = (140, 255, 255)

        # create masks
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        has_red = cv2.countNonZero(red_mask) > 500 # if 500 pixels are red/blue then there is red/blue
        has_blue = cv2.countNonZero(blue_mask) > 500 # avoids false positives
        
        detected_colors = []
        if has_red:
            detected_colors.append('red')
            frame[red_mask > 0] = [0, 0, 255] # highlight red
        if has_blue:
            detected_colors.append('blue')
            frame[blue_mask > 0] = [255, 0, 0] # highlight blue

        if detected_colors:
            self.color_pub.publish(String(data=str(detected_colors)))

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            self.get_logger().info(f"Detected ArUco markers: {ids.flatten().tolist()}")
            self.marker_pub.publish(String(data=str(ids.flatten().tolist())))

        processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(processed_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
