#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text')
        self.text_publisher = self.create_publisher(String, 'speech_input', 10)
        self.create_subscription(String, '/speaker/speech_input', self.speech_callback, 10)
        self.get_logger().info('Speech to Text Node initialized')
        self.get_logger().info('Listening on topic: /speaker/speech_input')
        self.get_logger().info('Publishing to topic: speech_input')

    def speech_callback(self, msg):
        """Process speech input and convert to text"""
        try:
            self.get_logger().info('Received raw message: %s' % msg.data)
            text_msg = String()
            text_msg.data = msg.data  # Simply pass through the text
            self.text_publisher.publish(text_msg)
            self.get_logger().info('Published message: %s' % text_msg.data)
        except Exception as e:
            self.get_logger().error('Speech processing error: %s' % str(e))
            self.get_logger().error('Error details:', exc_info=True)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        node.get_logger().info('Starting node spin')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error('Node error: %s' % str(e))
        node.get_logger().error('Error details:', exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
