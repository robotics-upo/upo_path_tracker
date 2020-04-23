#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

namespace Upo{
    namespace Utils{
        namespace Geometry{

            float getYawFromQuat(geometry_msgs::Quaternion quat);
            
        }
    }
}


#endif