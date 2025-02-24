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
from bike4py.client import LLMClient, ChatCompletionRequest, StatusEvent, CompletionEvent, ContentEvent
import json

class B4MBridge(Node):
    def __init__(self):
        super().__init__('b4m_bridge')
        
        # Initialize Bike4Mind API client
        colcon_prefix_path = os.environ.get('COLCON_PREFIX_PATH', '')
        if not colcon_prefix_path:
            self.get_logger().error('COLCON_PREFIX_PATH environment variable not set')
            raise RuntimeError('COLCON_PREFIX_PATH environment variable not set')
        
        # The first path in COLCON_PREFIX_PATH should be our install directory
        install_dir = colcon_prefix_path.split(':')[0]
        workspace_root = os.path.dirname(install_dir)
        token_file = os.path.join(workspace_root, 'b4m_api_token.txt')
        self.get_logger().info(f'Looking for token file at: {token_file}')
        try:
            with open(token_file, 'r') as f:
                token_data = json.load(f)
                refresh_token = token_data['state']['refreshToken']
                self.get_logger().info(f'Successfully read token from {token_file}')
        except FileNotFoundError:
            self.get_logger().error(f'b4m_api_token.txt not found at {token_file}. Please create this file with your B4M refresh token.')
            raise
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Error parsing token file: {str(e)}. The file should contain a JSON object with state.refreshToken')
            raise
        except Exception as e:
            self.get_logger().error(f'Error reading B4M refresh token: {str(e)}')
            raise

        if not refresh_token:
            self.get_logger().error('B4M refresh token cannot be empty')
            raise ValueError('B4M refresh token cannot be empty')
        
        # Get notebook ID from parameter or use default
        self.declare_parameter('notebook_id', 'b4m_robot')
        self.notebook_id = self.get_parameter('notebook_id').get_parameter_value().string_value
        
        self.get_logger().info('Creating B4M client...')
        self.b4m_client = LLMClient(refresh_token=refresh_token)
        
        # Get event loop from current thread or create new one
        try:
            self.event_loop = asyncio.get_event_loop()
        except RuntimeError:
            self.event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.event_loop)
        
        # Connect to websocket
        try:
            self.get_logger().info('Connecting to B4M websocket...')
            self.event_loop.run_until_complete(self.b4m_client.connect())
            self.get_logger().info('Successfully connected to B4M websocket')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to B4M websocket: {str(e)}')
            raise
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'b4m_action', 10)
        self.speech_pub = self.create_publisher(String, 'speech_output', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.create_subscription(String, '/speech_text', self.speech_callback, 10)
        self.create_subscription(CompressedImage, 'b4m/camera/image', self.vision_callback, 10)
        self.create_subscription(Odometry, 'odom', self.pose_callback, 10)
        
        # Start event processing
        self.event_processing_task = self.event_loop.create_task(self.process_events())
        
        self.get_logger().info('B4M Bridge node initialized')

    async def process_events(self):
        """Process events from the Bike4Mind API"""
        self.get_logger().info('Starting event processing loop')
        try:
            async for event in self.b4m_client.stream_events():
                self.get_logger().debug(f'Received event: {type(event)}')
                if isinstance(event, StatusEvent):
                    self.get_logger().info(f'Status update: {event.status}')
                elif isinstance(event, CompletionEvent):
                    self.get_logger().info(f'Completion: {event.success}')
                elif isinstance(event, ContentEvent):
                    content = event.content
                    self.get_logger().info(f'Content: {content}')
                    # Parse and execute actions from content
                    if ':<' in content and '>' in content:
                        await self.execute_action(content)
                    else:
                        self.get_logger().debug(f'Content does not contain action format: {content}')
        except Exception as e:
            self.get_logger().error(f'Error processing events: {str(e)}')
            # Try to reconnect
            try:
                self.get_logger().info('Attempting to reconnect to websocket...')
                await self.b4m_client.connect()
                self.get_logger().info('Successfully reconnected to B4M websocket')
            except Exception as e:
                self.get_logger().error(f'Failed to reconnect to B4M websocket: {str(e)}')
                
    async def process_message(self, b4m_message):
        """Process a B4M message using the Bike4Mind API client"""
        self.get_logger().info(f'Processing message: {b4m_message}')
        
        try:
            # Check if client is connected
            if not self.b4m_client.is_connected():
                self.get_logger().error('B4M client is not connected. Attempting to reconnect...')
                try:
                    await self.b4m_client.connect()
                    self.get_logger().info('Successfully reconnected to B4M websocket')
                except Exception as e:
                    self.get_logger().error(f'Failed to reconnect to B4M websocket: {str(e)}')
                    return
            
            # Create chat completion request
            request = ChatCompletionRequest(
                sessionId=self.notebook_id,
                message=b4m_message
            )
            
            # Submit the prompt
            self.get_logger().info(f'Submitting prompt to B4M client with notebook ID: {self.notebook_id}')
            try:
                await self.b4m_client.submit_prompt(request)
                self.get_logger().info('Successfully submitted prompt')
            except Exception as e:
                self.get_logger().error(f'Error submitting prompt: {str(e)}')
                return
            
        except Exception as e:
            self.get_logger().error(f'Error processing message with Bike4Mind API: {str(e)}')

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
                # Set the goal position
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                
                # Set the goal orientation (quaternion)
                goal_msg.pose.orientation.x = 0.0
                goal_msg.pose.orientation.y = 0.0
                goal_msg.pose.orientation.z = 0.0
                goal_msg.pose.orientation.w = 1.0
                
                self.goal_pub.publish(goal_msg)
                self.get_logger().info(f'Published navigation goal for waypoint {action_value}')
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                
        except Exception as e:
            self.get_logger().error(f'Error executing action: {str(e)}')

    def speech_callback(self, msg):
        """Handle incoming speech messages"""
        self.get_logger().info(f'Received speech: {msg.data}')
        b4m_message = f'HEAR:<{msg.data}>'
        future = asyncio.run_coroutine_threadsafe(self.process_message(b4m_message), self.event_loop)
        future.add_done_callback(lambda f: self.get_logger().info('Speech processing completed') if not f.exception() else self.get_logger().error(f'Speech processing failed: {f.exception()}'))

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
            # If we're within 0.5 meters of a waypoint, consider we're at it
            if current_pos.x < 0.5:
                b4m_message = f'AT_WAYPOINT:<1>'
                future = asyncio.run_coroutine_threadsafe(self.process_message(b4m_message), self.event_loop)
                future.add_done_callback(lambda f: self.get_logger().info('Pose processing completed') if not f.exception() else self.get_logger().error(f'Pose processing failed: {f.exception()}'))
                
        except Exception as e:
            self.get_logger().error(f'Error in pose callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create and run the node
    try:
        node = B4MBridge()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error running node: {str(e)}')
    finally:
        # Clean up
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
