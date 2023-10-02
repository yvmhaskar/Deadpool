import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class CoordSubVelPub(Node):
    def __init__(self):
        super().__init__('coord_sub_vel_pub')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel', 5)
        self.control_subscriber_ = self.create_subscription(Int32, 'control_input', self.control_callback, 5)
        self.twist_msg = Twist()
        self.control_input = 0 # just initial value

    def control_callback(self, msg):
        self.control_input = msg.data
    
    def update_movement_commands(self):
        avg_speed = 1
        self.twist.angular.z = avg_speed * self.control_input
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    coord_sub_vel_pub=CoordSubVelPub()
    rclpy.spin(coord_sub_vel_pub)
    
    coord_sub_vel_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
