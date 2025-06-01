import time
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

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

    # Log ¯\_(ツ)_/¯
    self.get_logger().info(f'GPIO Pin {self.output_pin} set to output.')
    self.get_logger().info(f'Stacklight Service Started.')

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
        
        actual_state = GPIO.input(self.output_pin)
        self.get_logger().info(f"Pin state (readback): {actual_state}")

        
        self.get_logger().info(f'Set light to {self.curr}')
      except:
        self.get_logger().info("Exception occurred ):")

def main(args = None):
    rclpy.init(args = args)

    stacklight_node = StackLightService()
    
    rclpy.spin(stacklight_node)

    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()