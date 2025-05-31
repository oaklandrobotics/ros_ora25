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
        self.publisher_ = self.create_publisher(Image, 'topic', 100)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Image()
        msg.data = self.subscription
        self.publisher_.publish(msg)
        self.get_logger().info('Published Image')

    def listener_callback(self, msg):
        self.get_logger().info('I heard Image')



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