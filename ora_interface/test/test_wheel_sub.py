import os
import sys
import time
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions

import rclpy

import can
from can import Message

# Launch feature node
def generate_test_description():
    # Grab current file path to find nodes
    file_path = os.path.dirname(__file__)

    # Set up the interface node
    interface_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, '..', 'ora_interface', 'interface_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{
        }]
    )

    # Set up the interface testing node
    interface_test_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(
            file_path, '..', 'ora_interface', 'interface_test_node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{
        }]
    )

    # Launch nodes and test
    return (
        launch.LaunchDescription([
            interface_node,
            interface_test_node,

            # Start tests
            launch_testing.actions.ReadyToTest(),
        ]),

        # Names for nodes
        {
            'interface': interface_node,
            'test': interface_test_node
        }
    )

# Test node

# 
# Unit Tests
#

class TestInterfaceTesterLink(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        # Initialize ROS context for test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown ROS context
        rclpy.shutdown()
    
    def test_wheel_transmit(self, wheel, proc_output):
        # List to store the messages
        msgs_received = []

        # Subscriber to place the messages
        sub = self.node.create_subscription(
            Message,
            'topic', # When the node is created this needs to be fixed
            lambda msg: msgs_received.append(msg), # Grabs the msg in a lambda then appends it to the list
            10
        )

        try:
            # Wait until talker transmits two messages over the topic
            end_time = time.time() + 10

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

                if len(msgs_received) > 2:
                    # At least 2 messages have been stored/sent
                    break
        
            # This tells us that the publisher is actually publishing messages to the topic
            self.assertGreater(len(msgs_received), 2)

            # Now we can see if those messages are actually correct
            for msg in msgs_received:
                proc_output.assertWaitFor(
                    expected_output = msg.data, process=interface_node
                )
        
        finally:
            # Destroy the testing node after we finish all of the tests
            self.node.destroy_subscription(sub)