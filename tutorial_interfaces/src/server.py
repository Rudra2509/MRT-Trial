import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from tutorial_interfaces.msg import MarkerArray  # Assuming the custom message is compiled and available

class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.bridge = CvBridge()

        # Define ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Subscriber for input images
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )

        # Publisher for detected marker information
        self.publisher = self.create_publisher(MarkerArray, 'aruco_markers', 10)

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Prepare data for custom message
            marker_array_msg = MarkerArray()

            # Store marker IDs
            marker_array_msg.ids = ids.flatten().tolist()

            # Store bounding box coordinates
            bboxes = []
            for corner in corners:
                # Flatten and collect bounding box coordinates
                flat_corner = [point for sub_corner in corner[0] for point in sub_corner]
                bboxes.append(flat_corner)
            marker_array_msg.bboxes = bboxes

            # Publish the marker data
            self.publisher.publish(marker_array_msg)

        else:
            self.get_logger().info("No markers detected.")

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoMarkerDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
