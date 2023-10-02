import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class ObjectTrackerSubscriber(Node):
    def __init__(self):
        super().__init__('object_tracker_subscriber')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 5)
        self.control_subscriber_ = self.create_subscription(Int32, 'direction', self.control_callback, 5)
        self.twist_msg = Twist()
        self.control_input = 0  # Default direction

    def control_callback(self, msg):
        self.control_input = msg.data

    def update_movement_commands(self):
        avg_speed = 1.0  # Adjust the speed as needed
        if self.control_input == 0:
            self.twist_msg.angular.z = 0.0  # Stop turning
        elif self.control_input == -1:
            self.twist_msg.angular.z = avg_speed  # Turn left
        elif self.control_input == 1:
            self.twist_msg.angular.z = -avg_speed  # Turn right
        else:
            self.twist_msg.angular.z = 0.0  # Default to stop turning
        self.publisher_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    object_tracker_subscriber = ObjectTrackerSubscriber()

    try:
        while rclpy.ok():
            object_tracker_subscriber.update_movement_commands()
            rclpy.spin_once(object_tracker_subscriber, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    object_tracker_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
