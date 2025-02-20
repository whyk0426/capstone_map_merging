import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TopicPulbisher(Node):
    def __init__(self):
        super().__init__('topic_pulisher')
        self.publisher_ = self.create_publisher(String, 'map_info', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'test_topic_test'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    topic_publiesr = TopicPulbisher()
    rclpy.spin(topic_publiesr)
    topic_publiesr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()