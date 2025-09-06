import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

class StackLightService(Node):
  def __init__(self):
    super().__init__('led_service')

    self.flashing = False
    timer_period = 0.5 # seconds

    # GPIO
    GPIO.setmode(GPIO.BOARD)
    self.output_pin = 7
    self.curr = GPIO.HIGH

    GPIO.setup(self.output_pin, GPIO.OUT, initial=GPIO.LOW)

    # Service toggles flash, timer actually flashes the light
    self.light_srv = self.create_service(Trigger, 'auto_light', self.toggle_flash)
    self.light_timer = self.create_timer(timer_period, self.flash_light)

    # Publisher for auton_mode (If light is flashing, send True, otherwise false)
    self.auton_state_pub = self.create_publisher(Bool, '/auton_mode', 10)
    self.auton_state_timer = self.create_timer(1.0, self.auton_state_callback)

    # Log ¯\_(ツ)_/¯
    # self.get_logger().info(f'GPIO Pin {self.output_pin} set to output.')
    # self.get_logger().info(f'Stacklight Service Started.')

  def toggle_flash(self, request, response):
    self.flashing = not self.flashing
    self.get_logger().info("Flashing toggled")

    if not self.flashing:
      GPIO.output(self.output_pin, GPIO.LOW)

    response.success = True
    response.message = f"Flashing is now {'ON' if self.flashing else 'OFF'}"

    return response

  def flash_light(self):
    if self.flashing:
      try:
        GPIO.output(self.output_pin, self.curr)
        self.curr = GPIO.LOW if self.curr == GPIO.HIGH else GPIO.HIGH
      except Exception as e:
        self.get_logger().warn("Exception occurred ):")
        self.get_logger().warn(e)

  def auton_state_callback(self):
    msg = Bool()
    msg.data = self.flashing
    self.auton_state_pub.publish(msg=msg)

def main(args = None):
    rclpy.init(args = args)

    stacklight_node = StackLightService()
    
    rclpy.spin(stacklight_node)

    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()