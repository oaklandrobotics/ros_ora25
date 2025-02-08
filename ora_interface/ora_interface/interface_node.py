import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32

# Subscribes to wheels/<left|right>/raw 
# Published to control_message

class interface_node(Node):
    def __init__(self):
        super().__init__('ora_interface')

        #Subscribers
        self.wheel_left_sub = self.create_subscription(Float32, 'wheels/left/raw', lambda msg: self.handle_wheel_desired(msg, 'left'), 10)        #2
        self.wheel_right_sub = self.create_subscription(Float32, 'wheels/right/raw', lambda msg: self.handle_wheel_desired(msg, 'right'), 10)      #3

        #Publishers
        # self.control_message_left = self.create_publisher(Control_Message, 'odrive_axis0/can_node/')
        # self.control_message_right = self.create_publisher(Control_Message, '/odrive_axis1/can_node/')
        # What else do we need??
        
        #Timer
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('CAN Node Started.')