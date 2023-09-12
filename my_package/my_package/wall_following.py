import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class WallFollow(Node):
    forward_distance = 0
    backward_distane = 0
    def __init__(self):
        super().__init__('wall_follow')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "stable_scan", self.scan_callback, 10)

    def scan_callback(self, scan: LaserScan):
        self.forward_distance = scan.ranges[45]
        self.backward_distane = scan.ranges[135]

    def run_loop(self):
        angular_speed = (self.forward_distance - self.backward_distane) / 2
        if angular_speed > 1:
            angular_speed = 1
        elif angular_speed < -1:
            angular_speed = -1
        print(angular_speed)     
        msg: Twist = Twist(angular=Vector3(x=0,y=0,z=angular_speed),linear=Vector3(x=.5,y=0,z=0))
        self.neato_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
