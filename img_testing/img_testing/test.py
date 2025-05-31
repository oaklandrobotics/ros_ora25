import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Subscribe to the img topic and get da img
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.listener_callback,
            100)
        self.subscription  # prevent unused variable warning

        # Create a publisher to publish the image we just got
        self.publisher_ = self.create_publisher(Image, 'topic/camera_info', 100)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.img = None

    def timer_callback(self):
        if self.img:
            msg = self.img

            #self.get_logger().info(self.subscription)
            self.publisher_.publish(msg)
            self.get_logger().info('Published Image')
        else:
            self.get_logger().error('img is none')

    def listener_callback(self, msg):
        self.img = msg
        self.get_logger().info('I heard Image ')


    def line_detection(self, img):
        # Figure out how to convert ros img/camera (find out which one is better) to cv2 image
        # Do your line detection algorithm
        # Mask your line algorithm on top of the depth image
        # Figure out how to publish the depth data to the costmap (Look up costmap 2d?)
        # Then ur done


        img = cv.imread('./Line_Detection/LineDetection2.png', cv.IMREAD_GRAYSCALE)
        assert img is not None, "Image file could not be read"

        edges = cv.blur(img, (5, 5))
        edges2 = cv.Canny(edges, 100, 200)
        edges3 = cv.Canny(edges, 100, 200)

        # Mask creation
        height, width = img.shape[:2]
        mask = np.zeros(edges3.shape[:2], np.uint8)
        test = cv.rectangle(edges3.copy(), (0, 0), (width, height // 3), 0, -1)
        masked_img = cv.bitwise_and(edges3, edges3, mask=mask)

        # Show static images
        # axs[0, 0].imshow(img, cmap='gray')
        # axs[0, 0].set_title('Original Image')
        # axs[0, 0].axis('off')

        # axs[1, 0].imshow(edges2, cmap='gray')
        # axs[1, 0].set_title('Edge Image')
        # axs[1, 0].axis('off')

        # axs[0, 1].imshow(test, cmap='gray')
        # axs[0, 1].set_title('Masked Edge Image')
        # axs[0, 1].axis('off')

        # Open video
        cap = cv.VideoCapture('./Line_Detection/testvid.mp4', cv.IMREAD_GRAYSCALE)
        if not cap.isOpened():
            print("Error opening video file")
            exit()

        # Read first frame to initialize plot
        ret, frame = cap.read()
        if not ret:
            print("Couldn't read video frame")
            cap.release()
            exit()

        # Convert BGR to RGB for matplotlib
        #frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        video_im = axs[1, 1].imshow(frame, cmap='gray')
        axs[1, 1].set_title('Video Frame')
        axs[1, 1].axis('off')

        plt.tight_layout()
        plt.ion()  # Turn on interactive mode
        plt.show()

        # Frame update loop
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            blur_image = cv.blur(frame_rgb, (5, 5))
            video_edges = cv.Canny(blur_image, 100, 200)
            height2, width2 = img.shape[:2]
            mask2 = np.zeros(video_edges.shape[:2], np.uint8)
            test2 = cv.rectangle(video_edges.copy(), (0, 0), (width2, height2 // 2), 0, -1)
            masked_img2 = cv.bitwise_and(video_edges, video_edges, mask=mask2)
            video_im.set_data(test2)  # Update image data
            plt.pause(0.001)  # Adjust delay to control playback speed

        cap.release()
        plt.ioff()  # Turn off interactive mode
        plt.show()





def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()