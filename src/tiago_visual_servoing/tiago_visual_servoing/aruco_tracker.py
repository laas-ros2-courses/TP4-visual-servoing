import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_250  # aruco_tag_30..110 ; aruco_tag_124 -> DICT_6X6_250


class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.bridge = CvBridge()
        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

        self.target_pub = self.create_publisher(PointStamped, '/aruco_tracker/target_pixel', 10)
        self.debug_pub = self.create_publisher(Image, '/aruco_tracker/debug_image', 10)
        self.create_subscription(Image, '/head_front_camera/image', self.image_callback, 10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, rejected = self.detector.detectMarkers(image)

        if ids is not None and len(ids) > 0:
            center = corners[0][0].mean(axis=0)
            u, v = float(center[0]), float(center[1])

            point = PointStamped()
            point.header = msg.header
            point.point.x = u
            point.point.y = v
            self.target_pub.publish(point)

            debug = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)
            cv2.circle(debug, (int(u), int(v)), 6, (0, 0, 255), -1)
        else:
            debug = image

        debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
