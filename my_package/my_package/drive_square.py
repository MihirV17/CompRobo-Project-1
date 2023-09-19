import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

# Create a class named DriveSquare that inherits from the Node class
class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')  # Initialize the ROS 2 node
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # Create a publisher for Twist messages
        self.create_timer(0.1, self.run_loop)  # Create a timer to run the control loop
        self.turns_executed = 0  # Initialize the number of turns executed
        self.executing_turn = False  # Initialize the flag for executing a turn
        self.side_length = 1  # Define the length of each side of the square
        self.time_per_side = 10  # Define the time to complete each side of the square
        self.time_per_turn = 2.0  # Define the time to complete a 90-degree turn
        self.start_time_of_segment = None  # Initialize the start time of the current segment

    # Function to start a new segment (side or turn)
    def start_segment(self):
        self.start_time_of_segment = self.get_clock().now()

    # Function to check if a segment is done based on duration
    def is_segment_done(self, duration):
        return self.get_clock().now() - self.start_time_of_segment > rclpy.time.Duration(seconds=duration)

    # Function to execute a 90-degree turn
    def execute_turn(self):
        msg = Twist()
        msg.angular.z = (pi / 4)  # Rotate at 45 degrees per second (90-degree turn in 2 seconds)
        self.vel_pub.publish(msg)

    # Function to drive straight (forward)
    def execute_straight(self):
        msg = Twist()
        msg.linear.x = self.side_length / self.time_per_side  # Drive forward at a constant speed
        self.vel_pub.publish(msg)

    # Control loop function
    def run_loop(self):
        if self.start_time_of_segment is None:
            self.start_segment()

        if self.executing_turn:
            segment_duration = self.time_per_turn  # Duration for a turn
        else:
            segment_duration = self.time_per_side  # Duration for driving straight (side)

        if self.is_segment_done(segment_duration):
            if self.executing_turn:
                self.turns_executed += 1  # Increment the count of executed turns
            self.executing_turn = not self.executing_turn  # Toggle the flag for executing a turn
            self.start_time_of_segment = None  # Reset the start time for the next segment
            print(self.executing_turn, self.turns_executed)  # Print current state information
        else:
            if self.executing_turn:
                self.execute_turn()  # Execute a turn
            else:
                self.execute_straight()  # Drive straight (forward)

# Main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python system
    node = DriveSquare()  # Create an instance of the DriveSquare class
    rclpy.spin(node)  # Start the ROS 2 node and spin (wait for callbacks)
    rclpy.shutdown()  # Shutdown the ROS 2 Python system when done

if __name__ == '__main__':
    main()  # Call the main function to run the drive square control node
