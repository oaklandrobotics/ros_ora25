import rclpy
from rclpy import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

class OdriveReInit(Node):
  def __init__(self):
    super().__init__('reinit_node')
    
    self.reinit_pub = self.create_publisher(Empty, '/odrive/reinit', 10)
    self.create_subscription("/joy", Joy, self.joy_callback, 10)
    
  def joy_callback(self, msg):
    # X or A (PS/XBOX) will reinitialize the control
    # Might change this to like start or something later depending on how it feels?
    if len(msg.buttons) > 0 and msg.buttons[0]:
      self.get_logger().info('Reinit button pressed')
      
      self.reinit_pub.publish(Empty())
    
def main(args=None):
  rclpy.init(args=args)
  
  reinit_node = OdriveReInit()
  
  rclpy.spin(reinit_node)
  
  reinit_node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()