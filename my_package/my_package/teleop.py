import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.key = None
        self.msg = Twist()

    def timer_callback(self):
        if self.key is not None:
            if self.key == 'w':
                print("Forward")
                self.msg.linear.x = 1.0
            elif self.key == 's':
                print("Backward")
                self.msg.linear.x = -1.0
            elif self.key == 'a':
                print("Left")
                self.msg.angular.z = (pi / 4)
            elif self.key == 'd':
                print("Right")
                self.msg.angular.z = -(pi / 4)
            else:
                self.msg.linear.x = 0
                self.msg.angular.z = 0

            self.publisher_.publish(self.msg)

    def run(self):
        while rclpy.ok():
            self.key = self.get_key()
            rclpy.spin_once(self)

    def get_key(self):
        try:
            import tty
            import sys
            import termios
            import select

            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0)

            if rlist:
                key = sys.stdin.read(1)
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
                return key

        except ImportError:
            pass

        return None

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopKeyboard()
    teleop_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
