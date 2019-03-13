#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <vector>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <navigator/navigator.hpp>

//Calbacks and variables used inside callbacks
bool laserGot;
sensor_msgs::LaserScan laserScan;

trajectory_msgs::MultiDOFJointTrajectory trajectory;
bool trajReceived;
geometry_msgs::PoseStamped nextPoint, globalGoal;

void laserScanCallback(const sensor_msgs::LaserScanConstPtr &scan);
void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj);
void globalGoalCallback(const geometry_msgs::PoseStampedConstPtr &globGoal_);

//Other stuff 
tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "navigator_node");
    ros::NodeHandle n;

    ros::Subscriber path_sub = n.subscribe("/trajectory_tracker/local_input_trajectory", 1000, trajectoryCallback);
    ros::Subscriber laser_sub = n.subscribe("/scanFront", 1000, laserScanCallback);
    ros::Subscriber global_goal_sub = n.subscribe("/move_base_simple/goal", 10, globalGoalCallback);

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(40);
    //First, create the security margin object, now we will start with defaults params
    Navigators::SecurityMargin securityMargin(&n);
    Navigators::Displacement despl(&n, &securityMargin, &laserScan, &tfBuffer);//Porque mierda me dice qe esta algo mal si compila?
    

    while (ros::ok())
    {
        ros::spinOnce();

    }
    return 0;
}
void laserScanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
    laserScan = *scan;
    laserGot = true;
}
void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
    trajectory = *trj;
    trajReceived = true;
    
    int i = 0;

    nextPoint.header.frame_id = "map";
    nextPoint.header.stamp = ros::Time(0);
    nextPoint.header.seq = rand();

    if (trajectory.points.size() > 1)
        i = 1;
    
    nextPoint.pose.position.x = trajectory.points[i].transforms[0].translation.x;
    nextPoint.pose.position.y = trajectory.points[i].transforms[0].translation.y;
    nextPoint.pose.orientation = trajectory.points[i].transforms[0].rotation;
}
//Used to calculate distance from base_link to global goal
void globalGoalCallback(const geometry_msgs::PoseStampedConstPtr &globGoal_)
{
    globalGoal = *globGoal_;
}