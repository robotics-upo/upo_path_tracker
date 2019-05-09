#ifndef SMOOTHER_H_
#define SMOOTHER_H_


#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>


using namespace std;

namespace Navigators{

class Smoother{

    typedef geometry_msgs::Twist Twist;

    public:
    Smoother(float lin_max_, float lin_min_, float acc_max_, float decc_max_, 
                            float angular_max_, float angular_acc_, float rate_);

    

    private:

    void loadParams(float lin_max_, float lin_min_, float acc_max_, float decc_max_, 
                            float angular_max_, float angular_acc_, float rate_);

    
    Twist Smooth(Twist vel);

    Twist capSpeed(Twist no_capped);

    float lin_max, lin_min;
    float acc_max, decc_max;
    float angular_max, angular_acc;

    float RATE;
    

    Twist prev;

};


};//namespace

#endif