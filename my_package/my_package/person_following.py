import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        ovr_time=.1
        self.speed_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.scan = 0
        self.timetick =self.create_timer(ovr_time,self.follow_person)
        

    def laser_callback(self, scan_data:LaserScan):
        min_distance =100
        range_check =.1
        
        for range_val in range(len(scan_data.ranges)):
            if scan_data.ranges[range_val] <min_distance and scan_data.ranges[range_val] >range_check:
                min_distance =scan_data.ranges[range_val]
                self.scan = range_val
                

    def follow_person(self):
        vel=Twist()
        baseline=0
        midline=100
        if baseline<self.scan<10 or 350 <self.scan<360:
            pass
        elif self.scan >midline:
            vel.angular.z=-.4
            
        else:
            vel.angular.z=.4
        if baseline<self.scan<60 or 300<self.scan<360:
            vel.linear.x=.2
        self.speed_pub.publish(vel)

    # def follow_person(self):
    #     if self.person_detected==True:
    #         self.send_robot_movement(self.linear_speed, 0.0)
    #         self.send_robot_movement(0.0,self.angular_speed,0.0)
    #     else:
    #         self.send_robot_movement(0.0, 0.0)

    # def send_robot_movement(self, linear_vel, angular_vel):
    #     twist = Twist()
    #     twist.linear.x = linear_vel
    #     twist.angular.z = angular_vel
    #     self.vel_pub.publish(twist)

def main():
        rclpy.init()
        follower = PersonFollower()
        rclpy.spin(follower)
        rclpy.shutdown()
        
        

if __name__ == '__main__':
    main()
