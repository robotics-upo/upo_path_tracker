#include "utils/geometry.hpp"

namespace Upo{
    namespace Utils{
        namespace Geometry{

            float getYawFromQuat(geometry_msgs::Quaternion quat)
            {
              double r, p, y;
              tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
              tf::Matrix3x3 M(q);
              M.getRPY(r, p, y);

              return y / M_PI * 180;
            }
            
        }
    } 
}