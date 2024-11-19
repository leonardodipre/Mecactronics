import rclpy
from geometry_msgs.msg import Twist
import time

def move_square():
    rclpy.init()
    node = rclpy.create_node('square_mover')
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    
    twist = Twist()

    for _ in range(4):  # 4 sides of the square
        # Move forward
        twist.linear.x = 2.0
        twist.angular.z = 0.0
        publisher.publish(twist)
        time.sleep(2)  # Move for 2 seconds
        
        # Turn 90 degrees
        twist.linear.x = 0.0
        twist.angular.z = 1.57
        publisher.publish(twist)
        time.sleep(1)  # Turn for 1 second
        
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    publisher.publish(twist)  # Stop
    node.destroy_node()
    rclpy.shutdown()

def main():
    move_square()

if __name__ == '__main__':
    main()
