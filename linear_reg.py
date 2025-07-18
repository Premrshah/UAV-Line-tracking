
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge 
import numpy as np

class linreg(Node):
    def __init__(self):
        super().__init__('linear_reg')
        self.get_logger().info("Node started")

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.annotated_img_pub = self.create_publisher(Image, 'final_image', 10)

    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # _, image = cv2.threshold(cv_image, 245, 255, cv2.THRESH_BINARY)
            # white_pixel_count = np.sum(image == 255)

            # if white_pixel_count > 50:
            #     kernel = np.ones((5, 5), np.uint8)
            #     dilation = cv2.dilate(image, kernel, iterations=3)
            #     m, b = self.calculate_regression(np.argwhere(dilation))
            #     x1, y1, x2, y2 = self.find_inliers(m, b, dilation.shape)

            #     if len(image.shape) == 2:
            #         image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

            #     cv2.line(image, (int(x1), int(y1)), (int(x2), int(y2)), color=(0, 255, 0), thickness=2)

            #     ros_image_msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
            #     self.annotated_img_pub.publish(ros_image_msg)
            #     ros_image_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            #     self.annotated_img_pub.publish(ros_image_msg)
            #     self.get_logger().info("Not enough of line detected")



            kernel = np.ones((15,15), np.uint8)
            lower_white = np.array([254, 254, 254])
            upper_white = np.array([255, 255, 255])


            dilated = cv2.dilate(cv_image, kernel, iterations=1)

            mask = cv2.inRange(dilated, lower_white, upper_white)
            filtered = cv2.bitwise_and(dilated, dilated, mask=mask)
            filtered_gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)

            _, thresh = cv2.threshold(filtered_gray, 127, 255, 0)
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            clean_mask = np.zeros_like(filtered_gray)
            visible_line = False

            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                length = cv2.arcLength(cnt, closed=True)
                if length < 300:
                    continue
                
                visible_line = True
                cv2.drawContours(clean_mask, [cnt], -1, 255, thickness=cv2.FILLED)

            result = cv2.bitwise_and(filtered_gray, clean_mask)

            ret, binary = cv2.threshold(result, 240, 255, cv2.THRESH_BINARY)
            if visible_line:
                points = np.argwhere(binary > 0)
                points = points[:, ::-1]  # swap (row,col) to (x,y)
                [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy, x0, y0 = [v[0] for v in (vx, vy, x0, y0)]  # extract scalars

                rows, cols = binary.shape
                lefty = int((-x0 * vy / vx) + y0)
                righty = int(((cols - x0) * vy / vx) + y0)

                result_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                cv2.line(result_color, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)

            else:
                result_color = cv2.cvtColor(filtered_gray, cv2.COLOR_GRAY2BGR)
            
            ros_image_msg = bridge.cv2_to_imgmsg(result_color, encoding='bgr8')
            self.annotated_img_pub.publish(ros_image_msg)
        except Exception as e:
            self.get_logger().info(str(e))

        self.get_logger().info("Callback function running")



    # def calculate_regression(self, points):
    #     points = points.astype(float)
    #     xs = points[:,1] #TODO
    #     ys = points[:,0] #TODO
    #     x_mean = np.average(xs) #TODO
    #     y_mean = np.average(ys) #TODO

    #     xy_mean = np.mean(np.multiply(xs, ys)) #TODO

    #     x_squared_mean = np.mean([x**2 for x in xs]) #TODO

    #     m = (x_mean * y_mean - xy_mean)/(x_mean ** 2 - x_squared_mean) #TODO

    #     b = y_mean - m * x_mean #TODO

    #     return (m,b)



    # def find_inliers(self, m, b, shape):
    #     y_max = shape[0]
    #     x_max = shape[1]

    #     for i in range(x_max):
    #         y = m*i+b
    #         if y < y_max:
    #             x1 = i
    #             y1 = y
    #             break

    #     for i in range(x_max-1,0,-1):
    #         y = m*i+b
    #         if y < y_max:
    #             x2 = i
    #             y2 = y
    #             break

    #     return x1, y1, x2, y2



def main(args=None):
    rclpy.init(args=args)
    node = linreg()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    