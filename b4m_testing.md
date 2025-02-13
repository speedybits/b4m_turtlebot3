
Test 1:
1) ros2 topic pub /speech_input std_msgs/msg/String "data: 'Please go to waypoint 1'" -1
2) Verify that the robot reaches waypoint 1

Terminal 1:
./b4m_launch.sh

Terminal 2:
source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic pub /speech_text std_msgs/msg/String "data: 'Please go to waypoint 1'" -1


Test 2:
1) Move robot toward the blue box
2) Take a JPG image of the blue box
3) Confirm that the b4m_bridge node has received the JPG image in the form of SEE:<image.jpg>
