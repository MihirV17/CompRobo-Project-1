import time  # Import the 'time' module for handling time-related operations
from geometry_msgs.msg import Twist  # Import Twist message type for robot movement
from sensor_msgs.msg import LaserScan  # Import LaserScan message type for laser data
import rclpy  # Import ROS 2 Python library
from rclpy.node import Node  # Import Node class for creating a ROS 2 node

# Create a class named ObstacleAvoidance that inherits from the Node class
class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')  # Call the constructor of the Node class
        ovr_time = 0.1  # Define a variable for the obstacle avoidance time interval
        self.speed_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # Create a publisher for Twist messages
        self.laser_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)  # Create a subscriber for LaserScan data
        self.scan = []  # Initialize an empty list to store laser scan data
        self.timetick = self.create_timer(ovr_time, self.avoid_obstacles)  # Create a timer to trigger obstacle avoidance

    # Callback function for processing laser scan data
    def laser_callback(self, scan_data: LaserScan):
        self.scan = scan_data.ranges  # Store the laser scan data in the 'scan' list

    # Function for obstacle avoidance
    def avoid_obstacles(self):
        vel = Twist()  # Create a Twist message for robot velocity
        forward_distance = 1.0  # Define the desired forward distance from obstacles

        # Check if the minimum distance in the front and rear of the robot is less than the desired distance
        if min(self.scan[0:10] + self.scan[350:360]) < forward_distance:
            # If too close to obstacles, set angular velocity (turn) based on which side has more space
            vel.angular.z = 0.4 if self.scan[0] > self.scan[359] else -0.4
            vel.linear.x = 0.0  # Set linear velocity (forward/backward) to stop
        else:
            # If no obstacles are too close, set both linear and angular velocities to move forward with slight turning
            vel.angular.z = 0.1
            vel.linear.x = 0.1

        # Publish the calculated velocity commands
        self.speed_pub.publish(vel)

# Main function
def main():
    rclpy.init()  # Initialize the ROS 2 Python system
    avoidance = ObstacleAvoidance()  # Create an instance of the ObstacleAvoidance class
    rclpy.spin(avoidance)  # Start the ROS 2 node and spin (wait for callbacks)
    rclpy.shutdown()  # Shutdown the ROS 2 Python system when done

# Entry point of the script
if __name__ == '__main__':
    main()  # Call the main function to run the obstacle avoidance node
