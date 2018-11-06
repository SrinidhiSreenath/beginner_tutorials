# ROS Publisher and Subscriber Nodes
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

A simple ROS package containing a pubisher and a subsriber node based on the [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).

## Dependencies
This is a ROS package which needs [ROS Kinetic](http://wiki.ros.org/kinetic) to be installed on Ubuntu 16.04. Please install ROS Kinetic and all its dependencies as outlined [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

## Build
In your desired directory, please run the following commands.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone git@github.com:SrinidhiSreenath/beginner_tutorials.git --branch Week10_HW
cd ..
catkin_make
```

## Run using rosrun
Open a terminal and run 
```
roscore
```
To launch the publisher node, run the following in a new terminal
```
cd <path to catkin_ws>
source devel/setup.bash
rosrun beginner_tutorials talker
```
To launch the subscriber node, run the following in a new terminal
```
cd <path to catkin_ws>
source devel/setup.bash
rosrun beginner_tutorials listener
```
To kill the running nodes, CTRL+C in the terminal or run the following in a new terminal 
```
rosnode kill talker
rosnode kill listener
```

## Run with launch file
To launch both the talker and listener node with a launch file, run the following command in a new terminal
```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials simpleTalkerAndListener.launch
```
The talker node will start in the terminal where the above commands are executed and listener node is launched in a new terminal.

To terminate both the nodes, CTRL+C in the talker node terminal.

## Modify publish message with a ROS service
A ROS service in this package can be utilised to change the message being published. To do this, launch both the talker and listener nodes either by rosrun or using roslaunch. In a new terminal, run the following commands:
```
cd <path to catkin_ws>
source devel/setup.bash
rosservice call /modify_output <desired message>
```

An example rosservice call:
```
rosservice call /modify_output "I am BATMAN!"
```
This modifies the published message from talker node and the change is reflected in the message the listener subscribes to as well.

## Modify loop rate in Publisher
The publsher node talker runs at 10 Hz frequency by default. If a different rate is desired, then launch the talker node as follows:

### Launching with rosrun
```
rosrun beginner_tutorials talker <desired frequency>
```
An example:
```
rosrun beginner_tutorials talker 7
```
If a non positive number is passed, the node throws an error and sets the frequency value to 10. If multiple values are entered, then the talker node shuts down.

### Launching with roslaunch
```
roslaunch beginner_tutorials simpleTalkerAndListener.launch frequency:=<desired value>
```
An example:
```
roslaunch beginner_tutorials simpleTalkerAndListener.launch frequency:=7
```
If a non positive number is passed, the node throws an error and sets the frequency value to 10.
