import rclpy
from rclpy.node import Node

# Subscribes to odrive_status, controller_status
# Also subscribes to the 
# Published to control_message

class interface_node(Node):
    def __init__(self):
        super().__init__('ora_interface')
        
        #