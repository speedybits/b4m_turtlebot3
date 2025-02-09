#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import speech_recognition as sr

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        
        # Declare parameters
        self.declare_parameter('use_simulation', True)
        self.use_simulation = self.get_parameter('use_simulation').value
        
        # Create publishers
        self.text_pub = self.create_publisher(String, '/speech_text', 10)
        self.status_pub = self.create_publisher(String, '/speech_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        if self.use_simulation:
            # Create subscriber for simulated speech input
            self.speech_sub = self.create_subscription(
                String,
                '/speaker/speech_input',
                self.speech_callback,
                10
            )
            self.get_logger().info('Voice Control Node initialized in SIMULATION mode')
        else:
            # Initialize speech recognizer for real microphone
            self.recognizer = sr.Recognizer()
            # Create a timer for continuous speech recognition
            self.timer = self.create_timer(1.0, self.process_microphone)
            self.get_logger().info('Voice Control Node initialized in REAL MICROPHONE mode')
    
    def process_microphone(self):
        """Process speech input from real microphone"""
        if not self.use_simulation:
            try:
                with sr.Microphone() as source:
                    self.publish_status("Listening...")
                    audio = self.recognizer.listen(source, timeout=5.0)
                    
                    try:
                        text = self.recognizer.recognize_google(audio)
                        self.process_speech_input(text)
                    except sr.UnknownValueError:
                        self.publish_status("Could not understand audio")
                    except sr.RequestError as e:
                        self.publish_status(f"Could not request results: {str(e)}")
            except Exception as e:
                self.publish_status(f"Error processing microphone: {str(e)}")
    
    def speech_callback(self, msg):
        """Process incoming simulated speech message"""
        if self.use_simulation:
            try:
                self.publish_status("Processing speech input...")
                self.process_speech_input(msg.data)
            except Exception as e:
                self.publish_status(f"Error processing speech: {str(e)}")
    
    def process_speech_input(self, text):
        """Common processing for both simulated and real speech input"""
        text = text.lower()
        self.publish_text(text)
        self.process_command(text)
    
    def publish_text(self, text):
        """Publish recognized text"""
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)
        self.get_logger().info(f'Recognized text: {text}')
        
    def publish_status(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {status}')
        
    def process_command(self, command):
        """Process recognized command and control the robot"""
        twist = Twist()
        command_recognized = True
        
        if 'forward' in command:
            twist.linear.x = 0.2
            self.get_logger().info('Command: Moving forward')
        elif 'backward' in command:
            twist.linear.x = -0.2
            self.get_logger().info('Command: Moving backward')
        elif 'left' in command:
            twist.angular.z = 0.5
            self.get_logger().info('Command: Turning left')
        elif 'right' in command:
            twist.angular.z = -0.5
            self.get_logger().info('Command: Turning right')
        elif 'stop' in command:
            # All values are already 0 by default
            self.get_logger().info('Command: Stopping')
        else:
            command_recognized = False
            self.get_logger().info('Command not recognized')
        
        if command_recognized:
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
