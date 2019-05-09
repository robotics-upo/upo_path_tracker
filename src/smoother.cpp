#include <navigator/smoother.hpp>


namespace Navigators{


typedef geometry_msgs::Twist Twist;

Smoother::Smoother(float lin_max_, float lin_min_, float acc_max_, float decc_max_, 
                            float angular_max_, float angular_acc_, float rate_){
    Smoother::loadParams(lin_max_, lin_min_,  acc_max_, decc_max_, angular_max_, angular_acc_, rate_);
}

void Smoother::loadParams(float lin_max_, float lin_min_, float acc_max_, float decc_max_, 
                            float angular_max_, float angular_acc_, float rate_){

    lin_max = lin_max_;
    lin_min = lin_min_;

    acc_max = acc_max_;
    decc_max = decc_max_;

    angular_max = angular_max_;
    angular_acc = angular_acc_;

    RATE = rate_;

    prev.linear.x = 0;
    prev.linear.y = 0;
    prev.linear.z = 0;

    prev.angular.x = 0;
    prev.angular.y = 0;
    prev.angular.z = 0;

    
}
Twist Smoother::Smooth(Twist vel){
    //First thing to do is to cap the incoming speed if it overcome the limits configured
    Twist speed = capSpeed(vel);
    
    //Now we have the "legal" version of the speed received
    if( vel.angular.z > prev.angular.z + angular_acc/RATE){
    
        speed.angular.z = vel.angular.z + angular_acc/RATE;

    }else if( vel.angular.z < prev.angular.z - angular_acc/RATE){
    
        speed.angular.z = vel.angular.z - angular_acc/RATE;
    
    }


    
}


Twist Smoother::capSpeed(Twist no_capped){

    Twist ret = no_capped;

    float mod = sqrtf( pow(no_capped.linear.x,2) + pow(no_capped.linear.y,2) );
    float theta = atan2(no_capped.linear.y, no_capped.linear.x);

    if(  mod > lin_max ){
        
        ret.linear.x = lin_max*cos(theta) * no_capped.linear.x/fabs(no_capped.linear.x);
        ret.linear.y = lin_max*sin(theta) * no_capped.linear.y/fabs(no_capped.linear.y);

    }else if( mod < lin_min ){

        ret.linear.x = lin_min*cos(theta) * no_capped.linear.x/fabs(no_capped.linear.x);
        ret.linear.y = lin_min*sin(theta) * no_capped.linear.y/fabs(no_capped.linear.y);

    }
    if( fabs(no_capped.angular.z) > angular_max ){
        ret.angular.z = angular_max* no_capped.angular.z/fabs(no_capped.angular.z);
    }
    
return ret;
}

}//namespace