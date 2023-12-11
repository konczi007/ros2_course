import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlesimController(Node):

    def __init__(self, reference_x):
        super().__init__('turtlesim_controller')

        self.declare_parameter('speed', 5.0) # Sebesség
        self.declare_parameter('omega', 40.0) # Szögsebesség
        self.declare_parameter('kp', 2.0)  # Arányossági konstans
        self.reference_x = reference_x # referenciapont


        self.pose = None
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10)

        self.twist_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def cb_pose(self, msg):
        self.pose = msg
        self.apply_proportional_control() # a megkapott helyzetet alapján a szabályozó szerint választja az irányt

# Az arányos szabályozás:
    def apply_proportional_control(self):
        if self.pose is not None:
            current_x = self.pose.x
            error = self.reference_x - current_x

            kp = self.get_parameter('kp').get_parameter_value().double_value
            control_output = kp * error # az irányítási jel

            twist = Twist()
            twist.linear.x = control_output
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)

    def go_straight(self, distance):
        speed = self.get_parameter('speed').get_parameter_value().double_value
        vel_msg = Twist()

        if distance > 0:
            vel_msg.linear.x = speed
        else:
            vel_msg.linear.x = -speed

        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0

        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        T = abs(distance / speed)  # s
        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)  # loop rate

        vel_msg.linear.x = 0.0
        self.twist_pub.publish(vel_msg)

    def turn(self, angle):
        omega = self.get_parameter('omega').get_parameter_value().double_value
        vel_msg = Twist()

        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0

        if angle > 0:
            vel_msg.angular.z = math.radians(omega)
        else:
            vel_msg.angular.z = math.radians(-omega)

        loop_rate = self.create_rate(100, self.get_clock())  # Hz
        T = abs(angle / omega)  # s
        self.twist_pub.publish(vel_msg)
        when = self.get_clock().now() + rclpy.time.Duration(seconds=T)

        while (self.get_clock().now() < when) and rclpy.ok():
            self.twist_pub.publish(vel_msg)
            rclpy.spin_once(self)  # loop rate

        vel_msg.angular.z = 0.0
        self.twist_pub.publish(vel_msg)

    def draw_koch_snowflake(self, order, length):
        for _ in range(3):
            self.draw_koch_curve(order, length)
            self.turn(-120)

    def draw_koch_curve(self, order, length):
        if order == 0:
            self.go_straight(length)
        else:
            self.draw_koch_curve(order - 1, length / 3)
            self.turn(60)
            self.draw_koch_curve(order - 1, length / 3)
            self.turn(-120)
            self.draw_koch_curve(order - 1, length / 3)
            self.turn(60)
            self.draw_koch_curve(order - 1, length / 3)

def main(args=None):
    rclpy.init(args=args)
    reference_x = 5.0  # vagy bármilyen más kezdeti érték
    tc = TurtlesimController(reference_x)

    # A Koch hópelyhet 3. rendig rajzoljuk ki és a szakasz hossza 3.0
    tc.draw_koch_snowflake(3, 3.0)

    # Destroy the node explicitly
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
