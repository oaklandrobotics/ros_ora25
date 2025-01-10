import rclpy, can, struct

from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from can.interface import Bus
from can import Message

# interface properties
can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'