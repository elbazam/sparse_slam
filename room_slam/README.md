# README #


### Acknowledgement ###

This work was done collaborating with Or Tslil and Tal Feiner, please follow their works.

### Requirements ###

- git


### What is this repository for? ###

Thesis work. Using sparse map representation for a less computational demanding SLAM process in large indoor environments.

### Video example

![video](media/faster_single_repitition.mp4)

### How do I get set up? ###

This package integrates with all the repositories in sparse_SLAM. Thus, you are required to install all of them following the instructions:

#### agent

- Copy the following line to your terminal:

```bash
$ cd ~/your_ws/src
$ git clone https://elbazam@bitbucket.org/elbazam/agent.git
$ cd ..
$ catkin_make
```
- Follow the instructions in https://bitbucket.org/elbazam/agent/src/master/

##### Python3 along python2

In robot description repository: 'https://github.com/TalFeiner/robot_description' run .sh file and follow the instructions.

#### Hector_SLAM

Install hector_slam in your catkin_workspace:
```bash
$ cd /your-working-ros-space/src
$ git clone -b melodic-devel https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
$ catkin_make
```

#### object_msgs

- Copy the following line to your terminal:

```bash
$ cd ~/your_ws/src
$ git clone https://elbazam@bitbucket.org/elbazam/object_msgs.git
$ cd ..
$ catkin_make
```

#### object_detector

- Copy the following line to your terminal:

```bash
$ cd ~/your_ws/src
$ git clone https://elbazam@bitbucket.org/elbazam/object_detector.git
$ pip3 install -e ./object_detector
$ catkin_make
```
- Follow the instructions in https://bitbucket.org/elbazam/object_detector/src/master/

#### exits_detector

- Copy the following line to your terminal:

```bash
$ cd ~/your_ws/src
$ git clone https://elbazam@bitbucket.org/elbazam/exits_detector.git
$ pip3 install -e ./exits_detector
$ catkin_make
```
- Follow the instructions in https://bitbucket.org/elbazam/exits_detector/src/master/

### Topics ###

#### Publishers
##### visualization_msgs.MarkerArray:

- '/SSLAM/objects' - objects visualization.
- '/SSLAM/rooms' - rooms visualization

##### visualization_msgs.Marker:

- '/SSLAM/corners' - Corner visualization
- '/SSLAM/exits' - Exit visualization
- '/SSLAM/walls' - walls visualization

#### geometry_msgs.PoseStamped

- '/SSLAM/pose' estimated agent's pose.

#### nav.msgs.Path

- '/SSLAM/path_history' - path topic with the estimated path documented
- '/SSLAM/real_path_history' - path topic with the real path documented
- '/hector/path_history' - path topic with the hector_slam estimated path documented

#### std_msgs.Int16

- '/Exp_index' - Number of experiment for comparison.

#### Subscribers
##### object_msgs.ObjectArray

- '/object_detector/objects' - list of located objects

##### object_msgs.LandmarkArray

- '/landmarks/corners' - list of located coreners

##### object_msgs.ExitLineArray

- '/landmarks/exits' - list of located exits

##### laser_line_extraction.LineSegmentList

- '/line_segments' - list of located walls

##### nav_msgs.Odometry

- '/odom' - encoder information.

##### geometry_msgs.PoseWithCovarianceStamped

- /initialpose - initial pose



## Topics flowchart

![flowchart](media/blocks_room.png)

