import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.run_loop)
        self.turns_executed = 0
        self.executing_turn = False
        self.side_length = 1
        self.time_per_side = 10
        self.time_per_turn = 2.0
        self.start_time_of_segment = None

    def start_segment(self):
        self.start_time_of_segment = self.get_clock().now()

    def is_segment_done(self, duration):
        return self.get_clock().now() - self.start_time_of_segment > rclpy.time.Duration(seconds=duration)

    def execute_turn(self):
        msg = Twist()
        msg.angular.z = (pi / 4) 
        self.vel_pub.publish(msg)

    def execute_straight(self):
        msg = Twist()
        msg.linear.x = self.side_length / self.time_per_side
        self.vel_pub.publish(msg)

    def run_loop(self):
        if self.start_time_of_segment is None:
            self.start_segment()

        if self.executing_turn:
            segment_duration = self.time_per_turn
        else:
            segment_duration = self.time_per_side

        if self.is_segment_done(segment_duration):
            if self.executing_turn:
                self.turns_executed += 1
            self.executing_turn = not self.executing_turn
            self.start_time_of_segment = None
            print(self.executing_turn, self.turns_executed)
        else:
            if self.executing_turn:
                self.execute_turn()
            else:
                self.execute_straight()

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
