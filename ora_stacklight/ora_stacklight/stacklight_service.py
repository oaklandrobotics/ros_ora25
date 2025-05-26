import time
import RPI.GPIO as GPIO
import rclpy
from rclpy.node import Node

class stacklight_service(Node):
    def __init__(self):
        super.init('led_service')
        self.srv = self.create_service(bool, 'auto_light', self.toggle_flash)
        self.flashing = False
        self.rate
        #NOTE: the physical pin on the Jetson that will be used is pin 12
        self.output_pin = 18
        self.flash_light()
        self.curr = GPIO.high
        GPIO.set_mode(GPIO.BCM)
        GPIO.setup(self.output_pin, GPIO.OUT, initial = GPIO.HIGH)

    def toggle_flash(self, request, response):
        self.flashing = not self.flashing
        self.get_logger.info("Flashing toggled")
        return self.flashing
    
    def flash_light(self):
        while True:
            if self.flashing:
                try:
                    while True:
                        time.sleep(.5)
                        GPIO.output(self.output_pin, curr)
                        curr ^= GPIO.HIGH
                finally:
                    GPIO.cleanup()

def main(args = None):
    rclpy.init(args = args)

    stacklight_service = stacklight_service()
    
    rclpy.spin(stacklight_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()