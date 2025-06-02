import rclpy, math, tf_transformations
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.declare_parameter('goal_distance', 1.0)
        self.goal_distance = self.get_parameter('goal_distance').value

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.goal_pub_period = 0.5
        self.timer = self.create_timer(self.goal_pub_period, self.publish_goal)

        self.current_pose = None

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def publish_goal(self):
        if self.current_pose is None:
            return
        
        # Get yaw from quaternion
        orientation_q = self.current_pose.orientation
        (_, _, yaw) = tf_transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Compute goal pose ahead
        x_goal = self.current_pose.position.x + self.goal_distance * math.cos(yaw)
        y_goal = self.current_pose.position.y + self.goal_distance * math.sin(yaw)

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x_goal
        goal.pose.position.y = y_goal
        goal.pose.position.z = 0.0

        # Maintain same heading
        goal.pose.orientation = self.current_pose.orientation

        self.goal_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)

    node = GoalPublisher()
    rclpy.spin()

    node.destroy_node()
    rclpy.shutdown()