
#include <ros/ros.h>
#include <simple_path_tracker/UpoNavigationSimplePathTracker.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_path_tracker");


    Upo::Navigation::SimplePathTracker tracker;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        tracker.navigate();
        loop_rate.sleep();
    }

    return 0;
}