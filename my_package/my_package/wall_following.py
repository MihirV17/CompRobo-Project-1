import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# Create a class named WallFollow that inherits from the Node class
class WallFollow(Node):
    forward_distance = 0.0  # Initialize forward distance to 0
    backward_distance = 0.0  # Initialize backward distance to 0
    
    def __init__(self):
        super().__init__('wall_following')  # Initialize the ROS 2 node
        timer_period = 0.1  # Define the timer's period (10 times per second)
        self.timer = self.create_timer(timer_period, self.run_loop)  # Create a timer to run the control loop
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)  # Create a publisher for Twist messages
        self.scan_sub = self.create_subscription(LaserScan, "stable_scan", self.scan_callback, 10)  # Create a subscriber for laser scan data
        
    # Callback function for processing laser scan data
    def scan_callback(self, scan: LaserScan):
        # Store forward and backward distances from the laser scan
        self.forward_distance = scan.ranges[45]
        print(f"Forward distance: {self.forward_distance}")
        self.backward_distance = scan.ranges[135]
        print(f"Backward distance: {self.backward_distance}")

    # Control loop function
    def run_loop(self):
        angular_speed = (self.forward_distance - self.backward_distance) * 5  # Calculate angular speed based on distances
        if angular_speed > 1.0:
            angular_speed = 1.0  # Limit the maximum angular speed to 1.0
        elif angular_speed < -1.0:
            angular_speed = -1.0  # Limit the minimum angular speed to -1.0
        print(angular_speed)  # Print the calculated angular speed

        # Create a Twist message with the calculated angular speed and a linear speed of 0.2
        msg: Twist = Twist(angular=Vector3(x=0.0, y=0.0, z=angular_speed), linear=Vector3(x=0.2, y=0.0, z=0.0))
        
        # Publish the Twist message to control the Neato robot
        self.neato_pub.publish(msg)

# Main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python system
    node = WallFollow()  # Create an instance of the WallFollow class
    rclpy.spin(node)  # Start the ROS 2 node and spin (wait for callbacks)
    rclpy.shutdown()  # Shutdown the ROS 2 Python system when done

if __name__ == '__main__':
    main()  # Call the main function to run the wall-following control node
