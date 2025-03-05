import rclpy, can

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


WHEEL_DIST = 28 / 39.37
WHEEL_DIA = 1 * 0.3048
WHEEL_RADIUS = WHEEL_DIA / 2

class WheelParser(Node):
    def __init__(self):
        super().__init__('twist_to_wheel')

        # subscribe to cmd_vel from robot
        self.cmd_vel_sub_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # publish desired speed for each wheel
        self.right_motor_pub_raw = self.create_publisher(Float32, 'wheels/right/raw', 10)
        self.left_motor_pub_raw = self.create_publisher(Float32, 'wheels/left/raw', 10)
        self.get_logger().info('Hello from twist_to_wheel')
        
    def cmd_vel_callback(self, vel):
        linear = vel.linear.x
        angular = vel.angular.z
        
        # Calculate the desired speed for each wheel
        speed_wish_right = (1 / WHEEL_RADIUS) * (linear + (WHEEL_DIST * angular) / 2)
        speed_wish_left = (1 / WHEEL_RADIUS) * (linear - (WHEEL_DIST * angular) / 2)
        
        # Publish the raw desired speed for each wheel
        self.right_motor_pub_raw.publish(Float32(data=speed_wish_right))
        self.left_motor_pub_raw.publish(Float32(data=speed_wish_left))
  
def main(args=None):
    rclpy.init(args=args)
    
    wheel_parser = WheelParser()
    
    rclpy.spin(wheel_parser)
    
    # Destroy
    wheel_parser.destroy_node()
    rclpy.shutdown()
        
    if __name__ == '__main__':
        main()