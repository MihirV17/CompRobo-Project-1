import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow')
        self.sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_distance = 0.5  # Desired distance from the wall

    def laser_callback(self, msg):
        # Define the range index for the direction you want to follow the wall (e.g., 0 degrees)
        target_index = 0
        desired_distance = self.target_distance

        if not msg.ranges:
            return

        if msg.ranges[target_index] > desired_distance:
            # If the distance to the wall is greater than the desired distance, turn towards it
            angular_z = -0.2  # Adjust the angular velocity as needed
            linear_x = 0.0
        else:
            # If the distance to the wall is less than the desired distance, turn away from it
            angular_z = 0.2  # Adjust the angular velocity as needed
            linear_x = 0.0

        # Create and publish a Twist message to control the robot's motion
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
