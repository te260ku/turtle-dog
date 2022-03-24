import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.listener_callback, 
            10)
        self.subscription
            
        self.br = CvBridge()

    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()