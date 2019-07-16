# Arco Path-Tracker

These classes are the ones used in the ARCO robot. The repository is composed of two classes and one ros node.

- nav_node: This is the main node, it instantiates two objects from the two classes mentioned above. 
- sfm_nav_node: An alternative navigation mode implementing a social force model. 

## Installation

You need the full ROS Kinetic installation. Follow the steps in http://wiki.ros.org/kinetic/Installation/Ubuntu and choose 

```
sudo apt-get install ros-kinetic-desktop-full
```

Once ROS is installed, inside your catkin workspace, src folder:

```
git clone https://github.com/robotics-upo/arco_path_tracker.git
```

Return to the catkin workspace base path and do

```
catkin_make
```

## Displacement class

This is main path-tracking class. It receives a MultiDOFTrajectory and send velocity commands to follow it in two ways:

### 1. Holonomic mode

### 2. Non-holonomic mode

## Security Margin class

It suscribe to one or two lasers topics depending which mode you select and create a virtual distance of security. It has functions to compare the distances returned by the lasers with the security margin it creates.

It creates two security perimeters. If some obstacle enters the inner security perimeter, the class continue returning "security stop state" until the obstacle has gone outside the outer perimeter. This is true for three possibles geometries: 


- Mode 0: It was designed to work with the old arco robot (the laundry machine) with the two lasers, the frontal and the rear one. 
- Mode 1: Full 360 square perimeter
- Mode 2: Full 360 elliptic security perimeter: Under construction

- [ ] Make easy to change between the 3 modes. 



### Parameters

Depending on the mode you want to use, you need to pass it differents paramters. Some of them are dynamically reconfigurables. 

#### Global parameters for all modes:

- security_mode: 0,1 or 2 depending on the geometry you want to use
- publish_markers: True to see nice markers in RViz
- front_laser_link_frame_id: Need explanation?
- stop_topic: The topic to which you want the class to publish a std_msgs::Bool with the current state

#### Parameters for mode 0
- only_front: To use or not front and rear security perimeters
- f_front: Eccentricity of the frontal perimeter
- f_back: Eccentricity of the rear perimeter
- inner_radius_front: 
- outer_radius_front:
- inner_radius_back:
- outer_radius_back:
- back_laser_link_frame_id: only if only_front is false
#### Parameters for mode 1
- robot_frame_id: Usually base_link
- delta_d: Distance from inner perimeter to outer perimeter. Outer perimeter is margin+delta_d
- margin_x: Inner distance in x direction
- margin_y: Inner distance in y direction

#### Parameters for mode 2

- f:
- inner_radius:
- outer_radius


## SFMNav Class

Local path tracker using a Social Force Model to adapt to people present in the environment.

It is a substitute to the Displacement class.

### Dependencies


You should previously install the lightsfm library in your system:

- Clone https://github.com/robotics-upo/lightsfm
- run make install there (the library is a set of .hpp files that will be used by the SFMNav node)


### Topics

Besides the topics published by Displacement, it adds a topic with markers for the social forces at:

/sfm/markers/robot_forces


