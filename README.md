# ROS- Implementation of Publisher node, Subscriber node, ROS Service Server node, Broadcasting TF frames, and Unit testing with Rostest.
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

A simple ROS package containing:
- a pubisher node and a subsriber node based on the [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).
- a service modifyOutput and the publisher node acting as a server node for service call. The service enables change in the string being published by the publisher node. 
- launch file to launch publisher node at desired frequency, the subscriber node and also enable/ disable recording of topics into a rosbag.
- broadcast a tf frame /talk with parent frame as /world with varying translation and a constant rotation in the publisher node. 
- test node to unit test the service call using the gtest framework.

## Dependencies
This is a ROS package which needs [ROS Kinetic](http://wiki.ros.org/kinetic) to be installed on Ubuntu 16.04. Please install ROS Kinetic and all its dependencies as outlined [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).

The package dependencies:

- catkin
- roscpp
- rospy
- std_msgs
- message_generation
- tf

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

## Broadcasting TF Frame
The talker node broadcasts a TF frame /talk with varying translation and a constant rotation with parent frame as /world.

To check the result of the frame being braodcasted, first run the launch file in a terminal -
```
roslaunch beginner_tutorials simpleTalkerAndListener.launch frequency:=7
```
### Echo TF in terminal
In a new terminal, using the tf_echo tool, check if the /talk frame is actually getting broadcast to tf:
```
rosrun tf tf_echo /world /talk
```
The terminal will output the translation and rotation of /talk frame at each time stamp. To terminate the output CTRL+C in the terminal.

### Visualize TF tree 
To visualise the tree of frames, use the rqt_tf_tree tool. Launch the nodes using the command above and in a new terminal run the below command.
```
rosrun rqt_tf_tree rqt_tf_tree
```
This will open a new window showing 2 frames /world and /talk with other information such as braocast frequency and the time of transform. To update the transform, refresh the window using the button on the top left. To close the window, CRTL+C in the terminal.

### Save TF tree in PDF
The tree can be saved as a PDF using the view_frames tool. In a new terminal, navigate to the desired directory where the PDF is to be generated and run the following command:
```
rosrun tf view_frames
```
This tool listens to the TF broadcast for 5 seconds and saves the result in a pdf. To view the result run the following command:
```
evince frames.pdf
```
This will open the PDF displaying the current transform tree. A [sample output pdf](https://github.com/SrinidhiSreenath/beginner_tutorials/blob/Week11_HW/results/tfframesbroadcastoutput.pdf) exists in the results directory of the package.

## Unit Testing with Rostest
Test nodes are written to unit test the ROS Service call. Two unit tests are written, one that tests the succesful existence of the ROS service modifyOutput and the second one to test the success of the service call.
To run the unit tests, execute the following in a new terminal
```
cd <path to catkin_ws>
catkin_make run_tests
```
The tests will be built and executed and the results will shown in the terminal. A sample result is shown below:
```
[ROSUNIT] Outputting test results to /home/srinidhi/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talkerTest.xml
[Testcase: testtalkerTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerTest/testInitializationOfROSService][passed]
[beginner_tutorials.rosunit-talkerTest/testROSServiceCall][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/srinidhi/.ros/log/rostest-srisbharadwaj-13430.log
-- run_tests.py: verify result "/home/srinidhi/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talkerTest.xml"
```

To run the unit tests using the launch file, run the following commands in the catkin workspace after all the packages are succesfully built.
```
cd <path to catkin_ws>
source devel/setup.bash
rostest beginner_tutorials talkerTest.launch 
```

## ROSBAG Recording
The published topics can be saved into a rosbag. The launch file has an argument to enable recording of the topics into a rosbag. To run the nodes and record the published topics, execute the folowing command in a terminal:
```
roslaunch beginner_tutorials simpleTalkerAndListener.launch rosbagRecord:=true
```
The messages in the /chatter topic will be recorded to a rosbag and saved. After ~15 seconds, CRTL+C in the terminal to terminate all the nodes. A [sample rosbag](https://github.com/SrinidhiSreenath/beginner_tutorials/blob/Week11_HW/results/chatter.bag) currently exists in the in the results subdirectory.

To test the rosbag, launch only the subscriber node using the commands outlined in the run section. Once the subscriber node is succesfully launched, in a new terminal navigate to the results subdirectory of the package and play the rosbag.
```
cd <path to catkin_ws>/src/beginner_tutorials/results
rosbag play chatter.bag
```
The rosbag will play the recorded messages in the /chatter topic and the subscriber will display the messages in its terminal. The bag will stop playing once its duration has ended.
