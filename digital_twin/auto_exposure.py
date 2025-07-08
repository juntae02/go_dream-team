import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class AutoExposureProcessor(Node):
    def __init__(self):
        super().__init__('auto_exposure_processor')
        self.bridge = CvBridge()
        
        # Subscribe to raw image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for processed image
        self.publisher_ = self.create_publisher(Image, '/auto_exposure_image', 10)

    def auto_exposure(self, frame):
        # Convert to YUV and equalize the Y channel (brightness)
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        processed = self.auto_exposure(frame)
        ros_image = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
        self.publisher_.publish(ros_image)
        self.get_logger().info('자동 노출 이미지 퍼블리시 완료')

def main(args=None):
    rclpy.init(args=args)
    node = AutoExposureProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
