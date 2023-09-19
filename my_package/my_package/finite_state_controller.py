import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from neato2_interfaces.msg import Bump  # Import custom message type for bump detection

# Create a class named WallFollow that inherits from the Node class
class WallFollow(Node):
    forward_distance = 0.0  # Initialize forward distance to 0
    backward_distance = 0.0  # Initialize backward distance to 0

    def __init__(self):
        super().__init__("finite_state_controller")  # Initialize the ROS 2 node
        timer_period = 0.1  # Define the timer's period (10 times per second)
        self.timer = self.create_timer(timer_period, self.run_loop)  # Create a timer to run the control loop
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)  # Create a publisher for Twist messages
        self.scan_sub = self.create_subscription(
            LaserScan, "stable_scan", self.scan_callback, 10
        )  # Create a subscriber for laser scan data
        self.bump_sub = self.create_subscription(Bump, "bump", self.bump_obstacle, 10)  # Create a subscriber for bump detection
        self.state = "wall_follower"  # Initialize the state to "wall_follower"
        self.counter = 0  # Initialize a counter for state transitions

    # Callback function for processing laser scan data
    def scan_callback(self, scan: LaserScan):
        self.forward_distance = scan.ranges[45]
        # print(f"forward distance {self.forward_distance}")
        self.backward_distance = scan.ranges[135]
        # print(f"back distance {self.backward_distance}")

    # Callback function for detecting bumps
    def bump_obstacle(self, bump: Bump):
        if bump.left_front or bump.right_front:
            self.state = "hit_obstacle"  # Transition to "hit_obstacle" state when a bump is detected

    # Control loop function
    def run_loop(self):
        # Wall follower state
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

        # Hit obstacle state
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
                self.state = "wall_follower"  # Transition back to "wall_follower" state
                self.counter = 0

# Main function
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python system
    node = WallFollow()  # Create an instance of the WallFollow class
    rclpy.spin(node)  # Start the ROS 2 node and spin (wait for callbacks)
    rclpy.shutdown()  # Shutdown the ROS 2 Python system when done

if __name__ == "__main__":
    main()  # Call the main function to run the finite state controller node
