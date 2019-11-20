
#include <ros/ros.h>
#include <navigator/displacement_2.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_node_2");


    Navigators::PathTracker tracker;
    //Displacement class subscribers

    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        tracker.navigate();
        loop_rate.sleep();
    }

    return 0;
}