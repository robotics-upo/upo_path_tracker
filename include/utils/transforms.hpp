#ifndef TRANSFORMS_H_
#define TRANSFORMS_H_

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace Upo{
    namespace Utils{
        namespace Transforms{

            geometry_msgs::PoseStamped transformPose(const trajectory_msgs::MultiDOFJointTrajectoryPoint &point,
                                                     const std::string &from, 
                                                     const std::string &to,
                                                     const tf2_ros::Buffer &tfBuffer);

            geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped &originalPose,
                                                     const std::string &from,
                                                     const std::string &to,
                                                     const tf2_ros::Buffer &tfBuffer);

            
        }
    }
}

#endif