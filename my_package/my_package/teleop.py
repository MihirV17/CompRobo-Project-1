import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
import sys
import termios
import tty
import select

class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.key = None

    def timer_callback(self):
        msg = Twist()
        if self.key is not None:
            if self.key == 'w':
                print("Forward")
                msg.linear.x = 1.0
            elif self.key == 's':
                print("Backward")
                msg.linear.x = -1.0
            elif self.key == 'a':
                print("Left")
                msg.angular.z = (pi / 4)
            elif self.key == 'd':
                print("Right")
                msg.angular.z = -(pi / 4)
            else:
                msg.linear.x = 0
                msg.angular.z = 0

            self.publisher_.publish(msg)

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        if rlist:
            key = sys.stdin.read(1)
            return key
        return None

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopKeyboard()

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            teleop_node.key = teleop_node.get_key()
            rclpy.spin_once(teleop_node)

    except Exception as e:
        print(f"Error: {str(e)}")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        rclpy.shutdown()

if __name__ == '__main__':
    main()
