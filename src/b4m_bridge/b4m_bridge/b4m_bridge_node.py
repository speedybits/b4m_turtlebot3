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
from bike4py.models import ChatCompletionParameters
import json
import traceback
import time
import threading

class B4MBridge(Node):
    def __init__(self):
        super().__init__('b4m_bridge')
        
        # Enable debug logging
        self.debug = True
        self.connected = False
        self.max_retries = 3
        self.connect_timeout = 10  # seconds
        
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
        self.declare_parameter('notebook_id', '67bb957759c6b05e1e12c1e6')
        self.notebook_id = self.get_parameter('notebook_id').get_parameter_value().string_value
        
        self.get_logger().info('Creating B4M client...')
        self.b4m_client = LLMClient(refresh_token=refresh_token)
        
        # Get event loop from current thread or create new one
        try:
            self.event_loop = asyncio.get_event_loop()
        except RuntimeError:
            self.event_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.event_loop)
        
        # Connect to websocket with retries
        connected = False
        retries = 0
        while not connected and retries < self.max_retries:
            try:
                self.get_logger().info(f'Connecting to B4M websocket (attempt {retries + 1}/{self.max_retries})...')
                
                # Create a timeout task
                connect_task = self.event_loop.create_task(self.b4m_client.connect())
                try:
                    self.event_loop.run_until_complete(
                        asyncio.wait_for(connect_task, timeout=self.connect_timeout)
                    )
                    connected = True
                    self.connected = True
                    self.get_logger().info('Successfully connected to B4M websocket')
                    break
                except asyncio.TimeoutError:
                    self.get_logger().error(f'Connection attempt {retries + 1} timed out after {self.connect_timeout} seconds')
                    # Cancel the connection task
                    connect_task.cancel()
                    try:
                        self.event_loop.run_until_complete(connect_task)
                    except asyncio.CancelledError:
                        pass
                
            except Exception as e:
                self.get_logger().error(f'Failed to connect to B4M websocket: {str(e)}')
            
            retries += 1
            if retries < self.max_retries:
                wait_time = min(5 * retries, 15)  # Exponential backoff, max 15 seconds
                self.get_logger().info(f'Waiting {wait_time} seconds before retry...')
                time.sleep(wait_time)
        
        if not connected:
            self.get_logger().error(f'Failed to connect after {self.max_retries} attempts')
            raise RuntimeError(f'Could not connect to B4M websocket after {self.max_retries} attempts')
        
        # Publishers
        self.action_pub = self.create_publisher(String, 'b4m_action', 10)
        self.speech_pub = self.create_publisher(String, 'speech_output', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.create_subscription(String, '/speech_text', self.speech_callback, 10)
        self.create_subscription(CompressedImage, 'b4m/camera/image', self.vision_callback, 10)
        self.create_subscription(Odometry, 'odom', self.pose_callback, 10)
        
        # Start event processing
        try:
            self.get_logger().info('Starting event processing task...')
            self.event_processing_task = self.event_loop.create_task(self.process_events())
            self.event_processing_task.add_done_callback(
                lambda f: self.get_logger().error(f'Event processing task ended: {f.exception()}') if f.exception() else None
            )
            # Run the event loop in a separate thread
            self.event_loop_thread = threading.Thread(target=self.event_loop.run_forever)
            self.event_loop_thread.daemon = True
            self.event_loop_thread.start()
            self.get_logger().info('Event processing task started')
        except Exception as e:
            self.get_logger().error(f'Failed to start event processing task: {str(e)}')
            raise
        
        self.get_logger().info('B4M Bridge node initialized')

    async def process_events(self):
        """Process events from the Bike4Mind API"""
        while True:
            try:
                # Check if we need to reconnect
                if not self.connected:
                    self.get_logger().info('Not connected to B4M websocket, attempting to reconnect...')
                    try:
                        await asyncio.wait_for(self.b4m_client.connect(), timeout=self.connect_timeout)
                        self.connected = True
                        self.get_logger().info('Successfully reconnected to B4M websocket')
                    except asyncio.TimeoutError:
                        self.get_logger().error(f'Reconnection attempt timed out after {self.connect_timeout} seconds')
                        self.connected = False
                        await asyncio.sleep(5)  # Wait before retrying
                        continue
                    except Exception as e:
                        self.get_logger().error(f'Failed to reconnect: {str(e)}')
                        self.connected = False
                        await asyncio.sleep(5)  # Wait before retrying
                        continue
                
                # Start streaming events
                if self.debug:
                    self.get_logger().info('Debug: About to start streaming events')
                    
                try:
                    if self.debug:
                        self.get_logger().info('Debug: Creating stream_events iterator')
                    event_stream = self.b4m_client.stream_events()
                    if self.debug:
                        self.get_logger().info('Debug: Created stream_events iterator')
                    
                    async for event in event_stream:
                        if self.debug:
                            self.get_logger().info(f'Debug: Received raw event: {event}')
                            self.get_logger().info(f'Debug: Event type: {type(event)}')
                            self.get_logger().info(f'Debug: Event dict: {event.__dict__}')
                        
                        self.get_logger().info(f'Received event type: {type(event).__name__}')
                        
                        if isinstance(event, StatusEvent):
                            self.get_logger().info(f'Status update: {event.status}')
                            # If we get a failure status, try to reconnect
                            if event.status == 'error' or event.status == 'failed':
                                self.get_logger().warn('Received error status, attempting to reconnect...')
                                try:
                                    await asyncio.wait_for(self.b4m_client.connect(), timeout=self.connect_timeout)
                                    self.connected = True
                                    self.get_logger().info('Successfully reconnected after error status')
                                except (asyncio.TimeoutError, Exception) as e:
                                    self.get_logger().error(f'Failed to reconnect after error status: {str(e)}')
                                    self.connected = False
                                    await asyncio.sleep(5)  # Wait before retrying
                        elif isinstance(event, CompletionEvent):
                            self.get_logger().info(f'Completion event received: success={event.success}, message={event.message}')
                            if not event.success:
                                self.get_logger().error(f'Completion failed: {event.message}')
                        elif isinstance(event, ContentEvent):
                            content = event.content
                            self.get_logger().info(f'Content: {content}')
                            # Parse and execute actions from content
                            if ':<' in content and '>' in content:
                                await self.execute_action(content)
                            else:
                                self.get_logger().info(f'Content does not contain action format: {content}')
                        else:
                            if self.debug:
                                self.get_logger().info(f'Debug: Unknown event details: {event.__dict__}')
                            self.get_logger().info(f'Unknown event type: {type(event).__name__}')
                            
                except Exception as e:
                    self.get_logger().error(f'Error in event stream loop: {str(e)}')
                    self.get_logger().error(f'Exception type: {type(e)}')
                    self.get_logger().error(f'Exception traceback: {traceback.format_exc()}')
                        
                if self.debug:
                    self.get_logger().info('Debug: Event stream ended, will try to reconnect')
                    
            except asyncio.CancelledError:
                self.get_logger().info('Event processing task cancelled')
                break
            except Exception as e:
                self.get_logger().error(f'Error processing events: {str(e)}')
                self.get_logger().error(f'Exception type: {type(e)}')
                self.get_logger().error(f'Exception traceback: {traceback.format_exc()}')
                # Try to reconnect
                try:
                    self.get_logger().info('Attempting to reconnect to websocket...')
                    await asyncio.wait_for(self.b4m_client.connect(), timeout=self.connect_timeout)
                    self.connected = True
                    self.get_logger().info('Successfully reconnected to B4M websocket')
                except (asyncio.TimeoutError, Exception) as e:
                    self.get_logger().error(f'Failed to reconnect to B4M websocket: {str(e)}')
                    self.connected = False
                    await asyncio.sleep(5)  # Wait before retrying

    async def process_message(self, b4m_message):
        """Process a B4M message using the Bike4Mind API client"""
        self.get_logger().info(f'Processing message: {b4m_message}')
        
        try:
            # Check if client is connected
            if not self.connected:
                self.get_logger().error('B4M client is not connected. Attempting to reconnect...')
                try:
                    await asyncio.wait_for(self.b4m_client.connect(), timeout=self.connect_timeout)
                    self.connected = True
                    self.get_logger().info('Successfully reconnected to B4M websocket')
                except Exception as e:
                    self.get_logger().error(f'Failed to reconnect to B4M websocket: {str(e)}')
                    return
            
            # Create chat completion request with required params
            request = ChatCompletionRequest(
                sessionId=self.notebook_id,
                message=b4m_message,
                params=ChatCompletionParameters(
                    model="gpt-4",  # Specify model
                    temperature=0.7,
                    max_tokens=1000,
                    stream=True
                )
            )
            
            if self.debug:
                self.get_logger().info(f'Debug: Created request with notebook ID: {self.notebook_id}')
                self.get_logger().info(f'Debug: Full request object: {request.__dict__}')
            
            # Submit the prompt and wait for response
            self.get_logger().info(f'Submitting prompt to B4M client with notebook ID: {self.notebook_id}')
            try:
                if self.debug:
                    self.get_logger().info('Debug: About to submit prompt')
                    
                response = await self.b4m_client.submit_prompt(request)
                
                if self.debug:
                    self.get_logger().info(f'Debug: Raw response from submit_prompt: {response}')
                    self.get_logger().info(f'Debug: Client connected: {self.connected}')
                
                self.get_logger().info(f'Successfully submitted prompt, response: {response}')
                
                if self.debug:
                    self.get_logger().info('Debug: Waiting for event stream to process...')
                
                # The response will be processed by the process_events method
                
            except Exception as e:
                self.get_logger().error(f'Error submitting prompt: {str(e)}')
                self.get_logger().error(f'Exception type: {type(e)}')
                self.get_logger().error(f'Exception traceback: {traceback.format_exc()}')
                return
                
        except Exception as e:
            self.get_logger().error(f'Error processing message with Bike4Mind API: {str(e)}')
            self.get_logger().error(f'Exception type: {type(e)}')
            self.get_logger().error(f'Exception traceback: {traceback.format_exc()}')

    def speech_callback(self, msg):
        """Handle incoming speech messages"""
        speech_text = msg.data
        self.get_logger().info(f'Received speech: {speech_text}')
        
        # Create task to process message
        try:
            if self.debug:
                self.get_logger().info('Debug: Creating task to process speech message')
                
            # Get event loop from current thread or create new one
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
            
            if self.debug:
                self.get_logger().info('Debug: Got event loop')
                self.get_logger().info(f'Debug: Event loop running: {loop.is_running()}')
            
            # Create and run task
            task = loop.create_task(self.process_message(f'HEAR:<{speech_text}>'))
            if not loop.is_running():
                if self.debug:
                    self.get_logger().info('Debug: Starting event loop')
                loop.run_until_complete(task)
            
        except Exception as e:
            self.get_logger().error(f'Error creating task to process speech: {str(e)}')
            self.get_logger().error(f'Exception type: {type(e)}')
            self.get_logger().error(f'Exception traceback: {traceback.format_exc()}')

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
                # Waypoint coordinates (example mapping)
                waypoint_coords = {
                    '1': (1.0, 0.0),
                    '2': (2.0, 0.0),
                    '3': (1.0, 1.0),
                    '4': (2.0, 1.0)
                }
                
                if action_value not in waypoint_coords:
                    self.get_logger().warn(f'Unknown waypoint: {action_value}')
                    return
                    
                x, y = waypoint_coords[action_value]
                
                # Set the goal position
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = 'map'
                goal_msg.header.stamp = self.get_clock().now().to_msg()
                
                # Set the goal position
                goal_msg.pose.position.x = x
                goal_msg.pose.position.y = y
                goal_msg.pose.position.z = 0.0
                
                # Set the goal orientation (quaternion)
                goal_msg.pose.orientation.x = 0.0
                goal_msg.pose.orientation.y = 0.0
                goal_msg.pose.orientation.z = 0.0
                goal_msg.pose.orientation.w = 1.0
                
                self.goal_pub.publish(goal_msg)
                self.get_logger().info(f'Published navigation goal for waypoint {action_value}: ({x}, {y})')
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                
        except Exception as e:
            self.get_logger().error(f'Error executing action: {str(e)}')
            self.get_logger().error(f'Exception type: {type(e)}')
            self.get_logger().error(f'Exception traceback: {traceback.format_exc()}')

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
