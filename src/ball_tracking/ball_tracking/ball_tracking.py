import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallTracker(Node):

    def __init__(self):
        super().__init__('ball_tracker')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.listener_callback, 
            10)
        self.subscription            

        self.br = CvBridge()

        self.linear = 0.0
        self.angular = 0.0

        self.target_width = 150
        

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher_.publish(msg)


    def listener_callback(self, data):
        frame = self.br.compressed_imgmsg_to_cv2(data)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # lower_color = np.array([0, 0, 0])
        # upper_color = np.array([179,128,100])
        lower_color = np.array([30, 64, 0])
        upper_color = np.array([90,255,255])

        mask = cv2.inRange(hsv, lower_color, upper_color)

 
        l = a = 0.0

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            hull = cv2.convexHull(contour)
            rect = cv2.boundingRect(hull)
            rects.append(np.array(rect))

        if len(rects) > 0:
            # 面積が最大の領域を抽出
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)

            x = rect[0]
            y = rect[1]
            w = rect[2]
            h = rect[3]
            cx = int(rect[0]+rect[2]/2)
            cy = int(rect[1]+rect[3]/2)

            cv2.drawMarker(frame, (cx, cy), color=(255, 0, 0), markerType=cv2.MARKER_TILTED_CROSS, thickness=2)


            # 目標位置との差分を計算する
            
            dw = 0.005 * (self.target_width - w)        # 前後方向
            dx = 0.005 * (640/2 - cx)   # 旋回方向
            
            # dw = self.target_width - w 
            # dx = 640/2 - cx   # 旋回方向


            # 前後方向
            # if abs(dw) > 10.0:
            #     if dw > 0:
            #         b = 0.05
            #     else:
            #         b = -0.05
            l = 0.0 if abs(dw) < 0.03 else dw   # ±10未満ならゼロにする
            # # 前後方向のソフトウェアリミッタ
            l = 0.05 if l > 0.05 else l
            l = -0.05 if l < -0.05 else l
            

            # 旋回
            # if abs(dx) > 20.0:
            #     if dx > 0:
            #         d = 0.3
            #     else:
            #         d = -0.3
            a = 0.0 if abs(dx) < 0.3 else dx   # ±20未満ならゼロにする
            # 旋回方向のソフトウェアリミッタ(±100を超えないように)
            a =  0.5 if a >  0.5 else a
            a = -0.5 if a < -0.5 else a


            print('dw=%f l=%f dx=%f a=%f'%(dw, l, dx, a))
            # print('dx=%f dw=%f'%(dx, dw))

        self.linear = l
        self.angular = a


        cv2.imshow("camera", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    ball_tracker = BallTracker()

    rclpy.spin(ball_tracker)

    ball_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
