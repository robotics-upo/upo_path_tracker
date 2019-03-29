
#include <ros/ros.h>
#include <math.h>
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>

bool go_home;

void goHomeCb(const std_msgs::Bool &msg);
//Other stuff
tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigator_node");
    ros::NodeHandle n;

    tf2_ros::TransformListener tfListener(tfBuffer);
    
    SecurityMargin securityMargin(&n);
    Navigators::Displacement despl(&n, &securityMargin, &tfBuffer);

    ros::Subscriber laser_sub = n.subscribe("/scanFront", 1000, &SecurityMargin::laser1Callback, &securityMargin);
    ros::Subscriber laser_sub2 = n.subscribe("/scanBackFiltered", 1000, &SecurityMargin::laser2Callback, &securityMargin);
    ros::Subscriber path_sub = n.subscribe("/trajectory_tracker/local_input_trajectory", 1, &Navigators::Displacement::trajectoryCb, &despl);
    ros::Subscriber global_goal_sub = n.subscribe("/move_base_simple/goal", 1, &Navigators::Displacement::globalGoalCb, &despl);
    //Homing under construction
    ros::Subscriber goHome = n.subscribe("/trajectory_tracker/go_home", 1, goHomeCb);

    ros::Rate loop_rate(40);

    while (ros::ok())
    {
        ros::spinOnce();
        
        securityMargin.canIMove();

        despl.navigate(0);        

        loop_rate.sleep();
    }

    return 0;
}
void goHomeCb(const std_msgs::Bool &msg)
{
    go_home = msg.data;
}