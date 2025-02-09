#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import subprocess
import sys
import signal
import os

def run_test():
    # Start voice control node
    voice_process = subprocess.Popen(
        ['ros2', 'run', 'b4m_voice', 'voice_control'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    try:
        # Initialize ROS
        rclpy.init()
        
        # Create our test node
        node = Node('b4m_voice_test')
        
        # Create publisher for simulated speech input
        pub = node.create_publisher(String, '/speaker/speech_input', 10)
        
        # Create a subscriber to check the status
        received_status = []
        def status_callback(msg):
            received_status.append(msg.data)
        
        sub = node.create_subscription(String, '/speech_status', status_callback, 10)
        
        # Wait for publishers and subscribers to be ready
        time.sleep(2)
        
        # Send test command
        msg = String()
        msg.data = 'forward'
        pub.publish(msg)
        
        # Wait and spin to receive response
        timeout = 5  # seconds
        start_time = time.time()
        while time.time() - start_time < timeout and not received_status:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        # Check results
        if received_status:
            print("PASS: Voice control node responded to command")
            return True
        else:
            print("FAIL: No response received from voice control node")
            return False
            
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        voice_process.terminate()
        voice_process.wait()

def main():
    try:
        success = run_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"Test failed with error: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main()
