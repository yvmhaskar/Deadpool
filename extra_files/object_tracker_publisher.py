import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class ObjectTrackerPublisher(Node):
    def __init__(self):
        super().__init__('object_tracker_publisher')
        self.publisher_ = self.create_publisher(Int32, 'direction', 5)
        self.control_input = 0  # Default direction

    def update_direction(self, direction):
        self.control_input = direction
        msg = Int32()
        msg.data = self.control_input
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    object_tracker_publisher = ObjectTrackerPublisher()

    try:
        while rclpy.ok():
            direction = int(input("Enter direction (-1, 0, 1): "))
            if direction in [-1, 0, 1]:
                object_tracker_publisher.update_direction(direction)
            else:
                print("Invalid direction. Please enter -1, 0, or 1.")
    except KeyboardInterrupt:
        pass

    object_tracker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
