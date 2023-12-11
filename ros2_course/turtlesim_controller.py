import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class KochSnowflakeBot(Node):
    def __init__(self):
        super().__init__('koch_snowflake_bot')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def draw_koch_snowflake(self, order, size):
        if order == 0:
            # Base case: Move the turtle forward
            self.move_turtle(size)
        else:
            # Recursive case: Draw a Koch snowflake
            size /= 3.0
            self.draw_koch_snowflake(order - 1, size)
            self.turn_turtle(-60)
            self.draw_koch_snowflake(order - 1, size)
            self.turn_turtle(120)
            self.draw_koch_snowflake(order - 1, size)
            self.turn_turtle(-60)
            self.draw_koch_snowflake(order - 1, size)

    def move_turtle(self, distance):
        msg = Twist()
        msg.linear.x = distance
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving forward: {distance}')

    def turn_turtle(self, angle_degrees):
        msg = Twist()
        msg.angular.z = math.radians(angle_degrees)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Turning: {angle_degrees} degrees')

def main(args=None):
    rclpy.init(args=args)

    koch_bot = KochSnowflakeBot()

    # Draw a Koch snowflake with order 3 and size 5
    koch_bot.draw_koch_snowflake(3, 5.0)

    rclpy.spin(koch_bot)

    koch_bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


