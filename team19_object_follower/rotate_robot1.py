import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class CoordSubscriber(Node):
    def __init__(self):
        super().__init__('coord_subscriber')
        self.subscription = self.create_subscription(Int32, 'direction', self.listener_callback, 5)
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

class VelPublisher(Node):
   def __init__(self):
      super().__init__('vel_publisher')
      self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 5)
      #self.timer = self.create_timer(0.5, self.publish_command)
      self.twist_msg = Twist()

   def publish_command(self):
      self.twist_msg.angular.z = 2 # this value needs to change to somehow account for input
      self.publisher_.publish(self.twist_msg)


def main(args=None):
    rclpy.init(args=args)
    # Subscriber
    coord_subscriber = CoordSubscriber()
    rclpy.spin(coord_subscriber)
    
    coord_subscriber.destroy_node()


    # publisher
    vel_publisher=VelPublisher()
    rclpy.spin(vel_publisher)
    vel_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
