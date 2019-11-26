
#include <ros/ros.h>
#include <navigator/PathTracker.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_node_2");


    PathTracker tracker;
    //Displacement class subscribers

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        tracker.navigate();
        loop_rate.sleep();
    }

    return 0;
}