# MyROS

Description:
This is a personal project with the goal of testing and developing new features for McGill Robotics
The current repository relies on ROS (Robot Operating System) 1 Noetic
The communication methods used are TCP (http, websockets), Srerial communication (rosserial) and webrtc
The build tool is catkin
Commonly used language are Python and Cpp with CMakelists.txt and XML config files
Hardware components testted: arduino uno, Tensys 4.0, Esp8266, Nvidia Jeston Nano, Thinkpad T490s

Dependencies:
- System requires native Ubuntu 20.04
- Ros-noetic installed on the system and added to source to the environment
- Use catkin build from the root workspace directory to rebuild the executables

Todo: 
- UI in react with rosbridge //development
- esp8266 rosserial regular and tcp //achieved
- dc motor control
- servo control
- multiple ros with micro ros and ros1
- research ros2
- stm32 ros integration

Running the project:
TO BE COMOPLETED

ROS Commands:
- roscd <package name>
- rosed <target_edit_file_name>
- rostopic list
- rostopic pub /topicName std_msgs/String "data: 'message here'"
- rostopic echo /topicName
- rosrun pkgName executable // can add --id (if using argparse for python)
- rosnode kill nodeName
- rosnode list
- rosmsg <msg_name>
- rosrun rosserial_python serial_node.py /dev/ttyUSB0
- rosrun rosserial_python serial_node.py tcp

Catkin Commands:
- catkin_make != catkin build
- catkin_create_pkg 
- catkin_make install (for building rosserial libraries in firmware)
