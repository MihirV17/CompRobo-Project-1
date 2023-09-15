import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from neato2_interfaces.msg import Bump


class WallFollow(Node):
    forward_distance = 0.0
    backward_distance = 0.0

    def __init__(self):
        super().__init__("finite_state_controller")
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "stable_scan", self.scan_callback, 10
        )
        self.bump_sub = self.create_subscription(Bump, "bump", self.bump_obstacle, 10)
        self.state = "wall_follower"
        self.counter = 0

    def scan_callback(self, scan: LaserScan):
        self.forward_distance = scan.ranges[45]
        # print(f"forward distance {self.forward_distance}")
        self.backward_distance = scan.ranges[135]
        # print(f"back distance {self.backward_distance}")

    def bump_obstacle(self, bump: Bump):
        if bump.left_front or bump.right_front:
            self.state = "hit_obstacle"

    def run_loop(self):
        # wall follower
        if self.state == "wall_follower":
            angular_speed = (self.forward_distance - self.backward_distance) * 5
            if angular_speed > 1.0:
                angular_speed = 1.0
            elif angular_speed < -1.0:
                angular_speed = -1.0
            print(f"angular speed {angular_speed}")
            msg: Twist = Twist(
                angular=Vector3(x=0.0, y=0.0, z=angular_speed),
                linear=Vector3(x=0.2, y=0.0, z=0.0),
            )
            self.neato_pub.publish(msg)
        # bump and spin
        if self.state == "hit_obstacle":
            print(f"counter {self.counter}")
            if self.counter < 5:
                msg: Twist = Twist(
                    angular=Vector3(x=0.0, y=0.0, z=0.0),
                    linear=Vector3(x=-0.5, y=0.0, z=0.0),
                )
                self.neato_pub.publish(msg)
                self.counter += 1
                print(f"hit: {self.state}")
            elif self.counter >= 5 and self.counter < 20:
                msg: Twist = Twist(
                    angular=Vector3(x=0.0, y=0.0, z=1.0),
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                )
                self.neato_pub.publish(msg)
                self.counter += 1
                print(f"hit_obstacle_angle {self.state}")
            else:
                self.state = "wall_follower"
                self.counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
