#include "utils/transforms.hpp"

namespace Upo{
    namespace Utils{
        namespace Transforms{

            geometry_msgs::PoseStamped transformPose(const trajectory_msgs::MultiDOFJointTrajectoryPoint &point,
                                                     const std::string &from, 
                                                     const std::string &to,
                                                     const tf2_ros::Buffer &tfBuffer)
            {
              geometry_msgs::PoseStamped pose;
              pose.header.frame_id = from;
              pose.header.seq = rand();
              pose.header.stamp = ros::Time::now();

              pose.pose.orientation = point.transforms[0].rotation;

              pose.pose.position.x = point.transforms[0].translation.x;
              pose.pose.position.y = point.transforms[0].translation.y;
              pose.pose.position.z = point.transforms[0].translation.z;
              
              return transformPose(pose, from, to, tfBuffer);
            }
            geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped &originalPose,
                                                     const std::string &from,
                                                     const std::string &to,
                                                     const tf2_ros::Buffer &tfBuffer)
            {
              geometry_msgs::TransformStamped transformStamped;
              geometry_msgs::PoseStamped nextPoseStamped;

              try
              {
                transformStamped = tfBuffer.lookupTransform(to, from, ros::Time(0));
              }
              catch (tf2::TransformException &ex)
              {
                ROS_WARN("No transform %s", ex.what());
              }

              tf2::doTransform(originalPose, nextPoseStamped, transformStamped);

              return nextPoseStamped;
            }
        }
    }
}