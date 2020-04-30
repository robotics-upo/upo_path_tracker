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
            double removeMultipleRotations(double rotation, double lower_bound, double upper_bound){
                
                if( std::fabs( (upper_bound - lower_bound) - 2*M_PI ) > 0.05 ){ //Check if they bounds are a full rotation
                    ROS_ERROR("You should give correct lower bounds and upper bounds values");
                }else{

                    while( rotation > upper_bound )
                        rotation -= 2 * M_PI;

                    while( rotation < lower_bound )
                        rotation += 2 * M_PI;
                }
                
                return rotation;
            }
        }
    } 
}