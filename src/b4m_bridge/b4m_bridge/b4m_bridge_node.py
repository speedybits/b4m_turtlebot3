#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import os
import math
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

class B4MBridge(Node):
    def __init__(self):
        super().__init__('b4m_bridge')
        
        # Load lookup table
        self.lookup_table = self.load_lookup_table()
        
        # Define waypoints (these should match your map)
        self.waypoints = {
            # Waypoint 1 is near the door
            '1': {'x': 2.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},  # Door location
            # Waypoint 2 is in the corner
            '2': {'x': 2.0, 'y': 2.0, 'z': 0.0, 'w': 1.0},  # Corner location
        }
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'b4m_action', 10)
        self.speech_pub = self.create_publisher(String, 'speech_output', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.create_subscription(String, '/speech_text', self.speech_callback, 10)
        self.create_subscription(CompressedImage, 'b4m/camera/image', self.vision_callback, 10)
        self.create_subscription(Odometry, 'odom', self.pose_callback, 10)
        
        # Set up TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('B4M Bridge node initialized')

    def load_lookup_table(self):
        lookup_dict = {}
        try:
            # Get the package share directory and construct path to lookup table
            pkg_dir = get_package_share_directory('b4m_bridge')
            lookup_file = os.path.join(pkg_dir, 'config', 'b4m_bridge_lookup_table.txt')
            
            self.get_logger().info(f'Looking for lookup table at: {lookup_file}')
            
            with open(lookup_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        input_msg, output_action = [x.strip() for x in line.split('===>')]
                        lookup_dict[input_msg] = output_action
            
            self.get_logger().info('Lookup table loaded successfully')
            self.get_logger().info(f'Lookup table contents: {lookup_dict}')
        except Exception as e:
            self.get_logger().error(f'Error loading lookup table: {str(e)}')
        
        return lookup_dict

    def process_message(self, b4m_message):
        """Process a B4M message using the lookup table"""
        self.get_logger().info(f'Processing message: {b4m_message}')
        
        # Convert lookup table keys to lowercase for case-insensitive matching
        lookup_dict_lower = {k.lower(): v for k, v in self.lookup_table.items()}
        
        # Look up the message in a case-insensitive way
        if b4m_message.lower() in lookup_dict_lower:
            action = lookup_dict_lower[b4m_message.lower()]
            self.get_logger().info(f'Found action: {action}')
            self.execute_action(action)
        else:
            self.get_logger().warn(f'No action found for message: {b4m_message}')

    def check_transforms(self):
        """Check if required transforms are available"""
        try:
            # Check map->odom transform
            self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            self.get_logger().info('map->odom transform available')
            
            # Check odom->base_link transform
            self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            self.get_logger().info('odom->base_link transform available')
            return True
            
        except TransformException as ex:
            self.get_logger().error(f'Could not transform: {str(ex)}')
            return False

    def execute_action(self, action):
        """Execute the appropriate action based on the lookup table output"""
        self.get_logger().info(f'Executing action: {action}')
        action_type = action.split(':<')[0]
        action_value = action.split(':<')[1][:-1]  # Remove trailing '>'
        
        msg = String()
        
        if action_type == 'SPEAK':
            msg.data = action_value
            self.speech_pub.publish(msg)
            self.get_logger().info(f'Published speech: {action_value}')
            
        elif action_type == 'GOTO_WAYPOINT':
            if not self.check_transforms():
                self.get_logger().error('Required transforms not available. Cannot navigate.')
                return
                
            if action_value in self.waypoints:
                waypoint = self.waypoints[action_value]
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                
                # Set the goal position
                goal_msg.pose.position.x = waypoint['x']
                goal_msg.pose.position.y = waypoint['y']
                goal_msg.pose.position.z = 0.0
                
                # Set the goal orientation (quaternion)
                goal_msg.pose.orientation.x = 0.0
                goal_msg.pose.orientation.y = 0.0
                goal_msg.pose.orientation.z = 0.0
                goal_msg.pose.orientation.w = waypoint['w']
                
                self.goal_pub.publish(goal_msg)
                self.get_logger().info(f'Published navigation goal for waypoint {action_value}: {waypoint}')
            else:
                self.get_logger().error(f'Unknown waypoint: {action_value}')

    def speech_callback(self, msg):
        """Handle incoming speech messages"""
        self.get_logger().info(f'Received speech: {msg.data}')
        b4m_message = f'HEAR:<{msg.data}>'
        self.process_message(b4m_message)

    def vision_callback(self, msg):
        """Handle incoming compressed images"""
        self.get_logger().info('Received compressed image')
        # Create a B4M message for the image
        b4m_message = 'SEE:<image.jpg>'
        self.process_message(b4m_message)

    def pose_callback(self, msg):
        """Handle robot pose updates"""
        # This could be used to track the robot's position and determine when
        # it has reached a waypoint, triggering an AT_WAYPOINT message
        pass

def main(args=None):
    rclpy.init(args=args)
    node = B4MBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
