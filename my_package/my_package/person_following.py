import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

# Create a class named PersonFollower that inherits from the Node class
class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')  # Initialize the ROS 2 node
        ovr_time = 0.1  # Define a variable for the time interval
        self.speed_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # Create a publisher for Twist messages
        self.laser_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)  # Create a subscriber for LaserScan data
        self.scan = 0  # Initialize the scan variable to store laser scan data
        self.timetick = self.create_timer(ovr_time, self.follow_person)  # Create a timer to trigger person following

    # Callback function for processing laser scan data
    def laser_callback(self, scan_data: LaserScan):
        min_distance = 100  # Initialize the minimum distance to a high value
        range_check = 0.1  # Define a threshold for valid range values
        
        # Iterate through the laser scan data to find the minimum distance
        for range_val in range(len(scan_data.ranges)):
            if scan_data.ranges[range_val] < min_distance and scan_data.ranges[range_val] > range_check:
                min_distance = scan_data.ranges[range_val]
                self.scan = range_val  # Store the index of the minimum distance

    # Function for following a person based on laser scan data
    def follow_person(self):
        vel = Twist()  # Create a Twist message for robot velocity
        baseline = 0
        midline = 100  # Define a midline value for deciding left or right movement
        
        # Check if the person is in the front or rear of the robot (avoidance behavior)
        if baseline < self.scan < 10 or 350 < self.scan < 360:
            pass  # Do nothing (avoid obstacles)
        # If the person is on the right, turn right
        elif self.scan > midline:
            vel.angular.z = -0.4
        # If the person is on the left, turn left
        else:
            vel.angular.z = 0.4
        
        # Move forward if the person is within a certain range in front or behind
        if baseline < self.scan < 60 or 300 < self.scan < 360:
            vel.linear.x = 0.2
        
        # Publish the calculated velocity commands
        self.speed_pub.publish(vel)

# Main function
def main():
    rclpy.init()  # Initialize the ROS 2 Python system
    follower = PersonFollower()  # Create an instance of the PersonFollower class
    rclpy.spin(follower)  # Start the ROS 2 node and spin (wait for callbacks)
    rclpy.shutdown()  # Shutdown the ROS 2 Python system when done

# Entry point of the script
if __name__ == '__main__':
    main()  # Call the main function to run the person following node
