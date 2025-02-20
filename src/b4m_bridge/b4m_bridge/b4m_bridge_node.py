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
import asyncio
from bike4py import LLMClient, ChatCompletionRequest, StatusEvent, CompletionEvent, ContentEvent

class B4MBridge(Node):
    def __init__(self):
        super().__init__('b4m_bridge')
        
        # Initialize Bike4Mind API client
        refresh_token = os.getenv('B4M_REFRESH_TOKEN')
        notebook_id = os.getenv('B4M_NOTEBOOK_ID')
        if not refresh_token or not notebook_id:
            self.get_logger().error('B4M_REFRESH_TOKEN and B4M_NOTEBOOK_ID environment variables must be set')
            raise ValueError('B4M_REFRESH_TOKEN and B4M_NOTEBOOK_ID environment variables must be set')
        
        self.b4m_client = LLMClient(refresh_token=refresh_token)
        self.notebook_id = notebook_id
        self.event_loop = asyncio.get_event_loop()
        
        # Connect to websocket
        self.event_loop.run_until_complete(self.b4m_client.connect())
        
        # Define waypoints (these should match your map)
        self.waypoints = {
            # Waypoint 1 is near the door
            '1': {'x': 2.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},  # Door location
            # Waypoint 2 is in the corner
            '2': {'x': 2.0, 'y': 2.0, 'z': 0.0, 'w': 1.0},  # Corner location
            # Blue box location
            'blue_box': {'x': 2.0, 'y': 1.0, 'z': 0.0, 'w': 1.0},  # Blue box location
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
        
        # Start event processing
        self.event_processing_task = self.event_loop.create_task(self.process_events())
        
        self.get_logger().info('B4M Bridge node initialized')

    async def process_events(self):
        """Process events from the Bike4Mind API"""
        try:
            async for event in self.b4m_client.stream_events():
                if isinstance(event, StatusEvent):
                    self.get_logger().info(f'Status update: {event.status}')
                elif isinstance(event, CompletionEvent):
                    self.get_logger().info(f'Completion: {event.success}')
                elif isinstance(event, ContentEvent):
                    content = event.content
                    self.get_logger().info(f'Content: {content}')
                    # Parse and execute actions from content
                    await self.execute_action(content)
        except Exception as e:
            self.get_logger().error(f'Error processing events: {str(e)}')

    async def process_message(self, b4m_message):
        """Process a B4M message using the Bike4Mind API client"""
        self.get_logger().info(f'Processing message: {b4m_message}')
        
        try:
            # Create chat completion request
            request = ChatCompletionRequest(
                sessionId=self.notebook_id,
                message=b4m_message
            )
            
            # Submit the prompt
            response = self.b4m_client.submit_prompt(request)
            self.get_logger().info(f'Submitted prompt: {response}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing message with Bike4Mind API: {str(e)}')

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

    async def execute_action(self, action):
        """Execute the appropriate action based on the API response"""
        self.get_logger().info(f'Executing action: {action}')
        
        try:
            # Parse action string to get type and value
            if ':<' not in action or '>' not in action:
                self.get_logger().warn(f'Invalid action format: {action}')
                return
                
            action_type = action.split(':<')[0]
            action_value = action.split(':<')[1].split('>')[0]
            
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
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                
        except Exception as e:
            self.get_logger().error(f'Error executing action: {str(e)}')

    def speech_callback(self, msg):
        """Handle incoming speech messages"""
        self.get_logger().info(f'Received speech: {msg.data}')
        b4m_message = f'HEAR:<{msg.data}>'
        asyncio.run_coroutine_threadsafe(self.process_message(b4m_message), self.event_loop)

    def vision_callback(self, msg):
        """Handle incoming compressed images"""
        self.get_logger().info('Received compressed image')
        # Currently not processing images, just logging the event
        pass

    def pose_callback(self, msg):
        """Handle robot pose updates"""
        # Check if we're near any waypoint and send AT_WAYPOINT message if we are
        try:
            current_pos = msg.pose.pose.position
            for waypoint_id, waypoint in self.waypoints.items():
                # Calculate distance to waypoint
                dx = current_pos.x - waypoint['x']
                dy = current_pos.y - waypoint['y']
                distance = math.sqrt(dx*dx + dy*dy)
                
                # If we're within 0.5 meters of a waypoint, consider we're at it
                if distance < 0.5:
                    b4m_message = f'AT_WAYPOINT:<{waypoint_id}>'
                    asyncio.run_coroutine_threadsafe(self.process_message(b4m_message), self.event_loop)
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error in pose callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = B4MBridge()
    rclpy.spin(node)
    node.event_loop.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
