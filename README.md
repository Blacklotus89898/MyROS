# MyROS

## Description:
This is a personal project with the goal of testing and developing new features for McGill Robotics
The current repository relies on ROS (Robot Operating System) 1 Noetic
The communication methods used are TCP (http, websockets), Srerial communication (rosserial) and webrtc
The build tool is catkin
Commonly used language are Python and Cpp with CMakelists.txt and XML config files
Hardware components testted: arduino uno, Tensys 4.0, Esp8266, Nvidia Jeston Nano, Thinkpad T490s

## Table of Content
- [Description](#description)
- [Dependencies](#dependencies)
- [Project Structure](#project-structure)
- [ROS Commands](#ros-commands)
- [Catkin Commands](#catkin-commands)
- [Todo](#todo)

## Dependencies:
- System requires native **Ubuntu 20.04**
- **ROS Noetic** installed on the system and added to source to the environment
- Use `catkin build` from the root workspace directory to rebuild the executables
- Require up-to-date npm and node
- Python3.8+


## Project Structure:
- Documentation
- react-ui-app: user interface and control
- my_controller: controller nodes for starting and stoping processes
- my_script: launch and script files
- my_embedded: embedded system code
- my_sandbox: experiemental environment

## ROS Commands:
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

## Catkin Commands:
- catkin_make != catkin build
- catkin_create_pkg 
- catkin_make install (for building rosserial libraries in firmware)

## Running the UI:
1. Clone the repo
2. `cd react-ui-app`
3. `npm i ; npm i react-router-dom`
4. `npm start`

## Running the ROS backend
1. Source the workspace
2. Launch rosbridge
3. Launch the servers related the to the feature commented in the UI componenets


## Todo
- [ ]  UI in react with rosbridge //development
- [ ] esp8266 rosserial regular and tcp //achieved
- [ ] dc motor control
- [ ] servo control
- [ ] multiple ros with micro ros and ros1
- [ ] research ros2
- [ ] stm32 ros integration
- [ ] CI/CD 
- [ ] Docker container

Running the project:
TO BE COMOPLETED