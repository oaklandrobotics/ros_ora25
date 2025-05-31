import rclpy
import message_filters
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

UPPER_BOUND = 255
#LOWER_BOUND = 110 #this changed for some reason
LOWER_BOUND = 200
#KERNEL = np.ones((5, 5), np.uint8)


class LinePerception(Node):
    def __init__(self):
        super().__init__('line_perception')
        self.declare_parameter('show_cv', rclpy.Parameter.Type.BOOL)
        self.bridge = CvBridge()
        self.img_raw_sub = message_filters.Subscriber(self, Image, '/depth_camera/image_raw')
        self.img_depth_sub = message_filters.Subscriber(self, Image, '/depth_camera/depth/image_raw')
        self.img_filtered_pub = self.create_publisher(Image, '/depth_camera/depth/image_raw_filtered', 10)
        self.ts = message_filters.ApproximateTimeSynchronizer((self.img_raw_sub, self.img_depth_sub), 10, slop=0.1)
        self.ts.registerCallback(self.img_callback)
        self.get_logger().info('Hello from line_perception')
        self.show_cv = self.get_parameter('show_cv').value
        if self.show_cv == True:
            self.get_logger().info('show_cv is True')

    def img_callback(self, img_raw: Image, img_depth: Image):
        cv_raw = self.bridge.imgmsg_to_cv2(img_raw, 'bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(img_depth, '32FC1')

        #Black and white
        bw = cv2.cvtColor(cv_raw, cv2.COLOR_BGR2GRAY)

        #Masking
        mask = np.zeros(cv_raw.shape[:2], dtype=np.uint8)
        cv2.rectangle(mask, (0, 360), (1280, 720), 255, -1) #adjust this for the real robot
        masked_image = cv2.bitwise_and(bw, bw, mask=mask)

        #Gaussian Blur
        blur = cv2.GaussianBlur(masked_image, (5, 5), 0)

        #Thresholding
        _, threshold = cv2.threshold(blur, LOWER_BOUND, UPPER_BOUND, cv2.THRESH_BINARY)

        #use threshold as a mask for the depth image
        masked = cv2.bitwise_and(cv_depth, cv_depth, mask=threshold)

        #Display the things
        SCALE = 2
        if self.show_cv == True:
            height, width = cv_raw.shape[0:2]
            raw_resized = cv2.resize(cv_raw, (int(width / SCALE), int(height / SCALE)))
            masked_normalized = cv2.normalize(masked, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            masked_color = cv2.cvtColor(masked_normalized, cv2.COLOR_GRAY2BGR)
            masked_resized = cv2.resize(masked_color, (int(width / SCALE), int(height / SCALE)))

            stack = cv2.hconcat([raw_resized, masked_resized])
            cv2.imshow('stack', stack)
        cv2.waitKey(1)

        #Publish the filtered image
        img_filtered = Image()
        img_filtered = self.bridge.cv2_to_imgmsg(masked, '32FC1')
        img_filtered.header = img_depth.header
        self.img_filtered_pub.publish(img_filtered)

def main():
    rclpy.init()
    
    line_perception = LinePerception()

    rclpy.spin(line_perception)

    line_perception.destroy_node()  
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()