
b4m_bridge

# Objective
We have created a ROS2 node called "b4m_bridge". The "b4m_bridge" node will receive sensor data from the robot, process it, and provide instructions for the robot to perform certain actions. The b4m_bridge will also deliver actions to the robot.

# Integration
The b4m_bridge node will subscribe to sensor data and publish instructions to the robot.

# b4m_bridge Node
Develop a ROS2 node called "b4m_bridge" which subscribes to sensor data from the robot. Sensor data includes: 
    text (from speech-to-text conversion)
    robot location (waypoint identifier)

The b4m_bridge node takes this sensor data and converts it to "B4M_messages" with the following format:
    speech-to-text ===> text received ===> HEAR:<text received>
    Robot arrived at Nav2 waypoint identifier ===> AT_WAYPOINT:<waypoint ID>

These "B4M_messages" are then sent as a message using the Bike4Mind API client as described here:
https://pypi.org/project/bike4py/

The response from the Bike4Mind API client is then processed by the b4m_bridge node and converted to an action to be published to the robot:
    SPEAK:<text to speak> ===> Instructs the robot to speak the specified text
    GOTO_WAYPOINT:<1> ===> Instructs the robot to go to the specified waypoint

# Waypoints
- Waypoints are pre-defined locations in the environment
- For now, there are 2 waypoints: 1 and 2 in the simulation.
- Waypoint 1 is near the door, and Waypoint 2 is near one of the corners of the room.

# Communication
Implement ROS2 topics or services for communication between the robot and the b4m_bridge node.

# Malformed Messages
In the case that a malformed message is received by the b4m_bridge node, the node should ignore the message and continue processing other messages.

# Topics/Services
Input Topic: Robot publishes sensor data (e.g., speech-to-text, positional data) to this topic.
Output Topic: b4m_bridge node publishes instructions for the robot (e.g., move commands, interaction cues) to this topic.
Message Types: Define relevant message structures, for example:
Sensor data: Image, Odometry, or custom message types.
Instructions: Twist (for movement), String (for simple commands), or custom types.

Connect Nodes: Use ROS2 launch files to start Webots and the b4m_bridge node together, ensuring they communicate via the defined topics.
Test Interactions: Simulate various scenarios in Webots to observe and refine how the b4m_bridge node processes data and directs the robot.