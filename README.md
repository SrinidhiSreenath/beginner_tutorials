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
git clone https://github.com/SrinidhiSreenath/beginner_tutorials.git
cd ..
catkin_make
```

## Run
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

