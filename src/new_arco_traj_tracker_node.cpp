#include <ros/ros.h>
#include <navigator/new_arco_traj_tracker.hpp>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "arco_traj_tracker_node");

  ArcoPathTracker apt_;
  ros::Rate loop_rate(10);

  while (ros::ok())
    {
        ros::spinOnce();
        apt_.navigate();
        loop_rate.sleep();
    }
	
  return 0;
}
