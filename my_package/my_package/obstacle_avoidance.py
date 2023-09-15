import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        ovr_time = 0.1
        self.speed_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.scan = []
        self.timetick = self.create_timer(ovr_time, self.avoid_obstacles)

    def laser_callback(self, scan_data: LaserScan):
        self.scan = scan_data.ranges

    def avoid_obstacles(self):
        vel = Twist()
        forward_distance = 1.0     
        if min(self.scan[0:10] + self.scan[350:360]) < forward_distance:
            vel.angular.z = 0.4 if self.scan[0] > self.scan[359] else -0.4
            vel.linear.x = 0.0
        else:
            vel.angular.z = 0.1
            vel.linear.x = 0.1
        self.speed_pub.publish(vel)
        
def main():
    rclpy.init()
    avoidance = ObstacleAvoidance()
    rclpy.spin(avoidance)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
