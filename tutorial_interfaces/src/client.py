import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tutorial_interfaces.msg import MarkerArray  # Custom message for marker detection results
from cv_bridge import CvBridge
import cv2

class ArucoClient(Node):
    def __init__(self):
        super().__init__('aruco_client')
        
        # Define a publisher to send the images to the server
        self.image_publisher = self.create_publisher(Image, 'image_raw', 10)
        
        # Define a subscriber to receive the marker data from the server
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            'aruco_markers',
            self.marker_callback,
            10
        )
        
        # For converting OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Example: Load an image with ArUco markers to be sent to the server
        self.send_image("path_to_your_aruco_image.png")

    def send_image(self, image_path):
        """Send an image to the server."""
        # Load the image
        cv_image = cv2.imread(image_path)

        if cv_image is None:
            self.get_logger().error(f"Failed to load image from {image_path}")
            return

        # Convert the image to a ROS Image message
        image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        # Publish the image message to the server
        self.image_publisher.publish(image_message)
        self.get_logger().info(f"Image sent to the server: {image_path}")

    def marker_callback(self, msg):
        """Handle the response from the server containing marker IDs and bounding boxes."""
        # Print the marker IDs
        self.get_logger().info(f"Detected Marker IDs: {msg.ids}")

        # Print the bounding box coordinates
        for i, bbox in enumerate(msg.bboxes):
            self.get_logger().info(f"Bounding box for marker {msg.ids[i]}: {bbox}")

def main(args=None):
    rclpy.init(args=args)
    aruco_client = ArucoClient()
    
    try:
        rclpy.spin(aruco_client)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
