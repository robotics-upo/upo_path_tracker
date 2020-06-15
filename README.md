# UPO Path Tracker Package

This packges comes from the old *arco_path_tracker* package used in the ARCO Robot. After that, the package has been reused for the NIx project, and a lot of changes were made, so after all, the package was a complete mess. To avoid that, we refactored it and the result is the current status. 

The goal of the current package structure is to allow people to reuse it or to add new nodes for new robots into it. Each node is located in one separated folder in the src/ and include/ folders. So if you want to add a new node, first create a branch and a create the corresponding folder with the new classes. 

You can also find the utils/ folder, in which we can put static methods shared by all the nodes, to reduce duplicated code. 

## Simple Path Tracker

This is the main node of the package. It's currently been used in the NIx project. It's a path tracker node for non-holonomic robots like Raposa. 

The node subscribes to the local paths published by the the local planner node from the [lazy_theta_star_planners](https://github.com/robotics-upo/lazy_theta_star_planners) package. If you want to use another planner, you should send the send message with the same structure, of adapt a new version of the path tracker. 

### Parameters and Configuration

It comes with standard default values in order to get a good behavior, but you know, sometimes you have to fine-tune these paramters. You will find below a list of the parameters, explaining the ones more complicated.

The node works with action lib, it means it has his own navigation action server so to send goals you have to create a client or send messages to the topic ```/Navigation/goal```. The name of the action server can be remapped from the launch file. 

#### Subscribed topics

 - Local path suscriber: ```/local_planner_node/local_path```

#### Published topics
 - Twist:  ```/nav_vel```
 - Speed markers for rviz:  ```/speed_markers```

#### Services

 - Check rotation client: Client from the service server in the custom costmap node to check if a rotation is possible depending of the surrounding obstacles
 - Reset Costmap client: Client to clean the costmap when starting the approximation to the goal, in order to avoid errors or problem when costmap is dirty

#### Parameters

 - *linear_max_speed*: 
 - *linear_max_speed_back*: The backwards max speed
 - *angular_max_speed*
 - *angle_margin*: In the final approximation maneouvre, the angular difference with the goal orientation for the robot to consider that it has arrived to the goal. 
 - *start_aproximation_distance*: The distance to start with the approximation phase.
 - *a*: Exponential speed factor for the angular speed.
 
![\omega_z = \pm \omega_{max} (1 - e^{-a \cdot \Delta \alpha}) ](https://render.githubusercontent.com/render/math?math=%5Comega_z%20%3D%20%5Cpm%20%5Comega_%7Bmax%7D%20(1%20-%20e%5E%7B-a%20%5Ccdot%20%5CDelta%20%5Calpha%7D)%20)

 With \\( \Delta \alpha \\) the angular difference to the goal orientation and the sign depending on the rotation direction. 

 - *b*: Exponential speed factor for the linear speed. The same as above but for the linear speed.
 - *b_back*: The same as *b* but for the backwards displacement. 
 - *dist_aprox1_*: The linear approximation distance to the goal
 - *local_paths_timeout*: The timeout time of the paths
 - *rot_thresh*: This is a number related to the quantity of the obstacles surrounding the robot and the robot radius. This is evaluated when doing the angular approximation to the goal.
 - *robot_base_frame*: Robot frame id: base_link by default
 - *world_frame_id*: World frame id: map by default
 - *odom_frame*: Odometric frame id: odom by default
 - *rate*: The running rate of the main process. 
 - *backwards_duration*: When the robot finds that it can't rotate, he will try to achieve the goal by going backwards. The backwards duration is the time it will go backwards without checking if it can rotate to go again forwards.


### TODOs

 - [] Add odometric path tracking
 - [] Fix some backwards bad behavior that results sometimes 
 - [] Implement rotation in place action
 - [] Integrate with the security check package

## SFM Path Tracker

Very old, not used right now, we should adapt it to the action server communication protocol

## ARCO Path Tracker

Very old, not used right now, we should adapt it to the action server communication protocol

## Dependencies

For the social force model you need to install the social force navigation library:
