
#include <simple_path_tracker/UpoNavigationSimplePathTracker.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_path_tracker");

    Upo::Navigation::SimplePathTracker tracker;

    ros::spin();

    return 0;
}