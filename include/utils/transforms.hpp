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
            
            /**
             * @brief 
             * 
             * @param point 
             * @param from 
             * @param to 
             * @param tfBuffer 
             * @return geometry_msgs::PoseStamped 
             */
            geometry_msgs::PoseStamped transformPose(const trajectory_msgs::MultiDOFJointTrajectoryPoint &point,
                                                     const std::string &from, 
                                                     const std::string &to,
                                                     const tf2_ros::Buffer &tfBuffer);
            /**
             * @brief 
             * 
             * @param originalPose 
             * @param from 
             * @param to 
             * @param tfBuffer 
             * @return geometry_msgs::PoseStamped 
             */
            geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped &originalPose,
                                                     const std::string &from,
                                                     const std::string &to,
                                                     const tf2_ros::Buffer &tfBuffer);
            
            /**
             * @brief Get the Current Orientation object
             * 
             * @param base_frame 
             * @param reference_frame 
             * @return tf2::Matrix3x3 &  
             */
            tf2::Matrix3x3 &getCurrentOrientation(const tf2_ros::Buffer &tf_buffer,
                                                  const std::string &base_frame = "base_link",
                                                  const std::string &reference_frame = "world" );
            
        }
    }
}

#endif