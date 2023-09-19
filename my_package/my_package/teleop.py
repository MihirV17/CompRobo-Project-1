import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
import sys
import termios
import tty
import select

# Create a class named TeleopKeyboard that inherits from the Node class
class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')  # Initialize the ROS 2 node
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # Create a publisher for Twist messages
        self.timer_ = self.create_timer(0.1, self.timer_callback)  # Create a timer for sending commands
        self.key = None  # Initialize the key variable to store user input

    # Timer callback function for sending Twist messages
    def timer_callback(self):
        msg = Twist()  # Create a Twist message for robot velocity
        if self.key is not None:
            if self.key == 'w':
                print("Forward")
                msg.linear.x = 1.0
            elif self.key == 's':
                print("Backward")
                msg.linear.x = -1.0
            elif self.key == 'a':
                print("Left")
                msg.angular.z = (pi / 4)  # Rotate left at 45 degrees per second
            elif self.key == 'd':
                print("Right")
                msg.angular.z = -(pi / 4)  # Rotate right at 45 degrees per second
            else:
                msg.linear.x = 0
                msg.angular.z = 0

            self.publisher_.publish(msg)  # Publish the calculated velocity commands

    # Function to get a single key press from the user
    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0)  # Check if there is input from the keyboard
        if rlist:
            key = sys.stdin.read(1)  # Read a single character from the keyboard
            return key
        return None

# Main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python system
    teleop_node = TeleopKeyboard()  # Create an instance of the TeleopKeyboard class

    try:
        tty.setraw(sys.stdin.fileno())  # Set the terminal to raw mode to capture single key presses
        while rclpy.ok():
            teleop_node.key = teleop_node.get_key()  # Get the user's key input
            rclpy.spin_once(teleop_node)  # Spin once to process the key input

    except Exception as e:
        print(f"Error: {str(e)}")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))  # Reset the terminal settings
        rclpy.shutdown()  # Shutdown the ROS 2 Python system

if __name__ == '__main__':
    main()  # Call the main function to run the teleop keyboard control node
