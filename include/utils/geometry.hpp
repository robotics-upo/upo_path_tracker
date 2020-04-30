#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

namespace Upo{
    namespace Utils{
        namespace Geometry{

            /**
             * @brief Get the Yaw From Quat object
             * 
             * @param quat 
             * @return float 
             */
            float getYawFromQuat(geometry_msgs::Quaternion quat);
            /**
             * @brief 
             * 
             * @param x0 
             * @param y0 
             * @param x 
             * @param y 
             * @return double 
             */
            inline double euclideanDistance(double x0, double y0, double x, double y)
            {
              return sqrtf(pow(x - x0, 2) + pow(y - y0, 2));
            }
            /**
             * @brief 
             * 
             * @param pose1 
             * @param pose2 
             * @return double 
             */
            inline double euclideanDistance(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
            {
              return euclideanDistance(pose1.pose.position.x, pose1.pose.position.y, 
                                       pose2.pose.position.x, pose2.pose.position.y);
            }
            /**
             * @brief 
             * 
             * @param next 
             * @return double 
             */
            inline double euclideanDistance(geometry_msgs::PoseStamped next)
            {
              return euclideanDistance(0, 0, next.pose.position.x, next.pose.position.y);
            }
            /**
             * @brief 
             * 
             * @param angle 
             * @return float 
             */
            inline double deg2Rad(double angle)
            {
              return angle / 180.0 * M_PI;
            }
            /**
             * @brief 
             * 
             * @param angle 
             * @return float 
             */
            inline double rad2Deg(double angle)
            {
              return angle / M_PI * 180.0;
            }
            /**
             * @brief It subtracts n times 360 degrees until the rotation is in the interval [-M_PI, M_PI]
             * 
             * @param rotation Original rotation (RADIANS)
             * @return double Reduced rotation (RADIANS) 
             */
            double removeMultipleRotations(double rotation, double lower_bound = -M_PI , double upper_bound = M_PI);//TODO Not tested
        }
    }
}

#endif