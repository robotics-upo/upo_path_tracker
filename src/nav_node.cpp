
#include <ros/ros.h>
#include <math.h>
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>

//Calbacks and variables used inside callbacks
sensor_msgs::LaserScan laserScan;

sensor_msgs::LaserScanConstPtr laserScanCPtr;

trajectory_msgs::MultiDOFJointTrajectoryPoint nextPoint;
geometry_msgs::PoseStamped globalGoal;

bool trajReceived, laserGot, status;

void laserScanCallback(const sensor_msgs::LaserScanConstPtr &scan);
void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &trj);
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

    //First, create the security margin object, now we will start with defaults params
    SecurityMargin securityMargin(&n);
    //We create the displacement object and we pass it the objects to work with
    Navigators::Displacement despl(&n, &securityMargin, &laserScan , &tfBuffer); 

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        securityMargin.publishRvizMarkers();
        
        //Used to test if security margin works 
        /*if(laserGot){
            if(securityMargin.canIMove(&laserScan)){
                ROS_ERROR("ME PUEDO MOVER");
            }   
        }
        */
       // ! This works 
        //if(laserGot)
        //    securityMargin.read(laserScanCPtr);
        /*if(laserGot){
            if(!securityMargin.checkObstacles(0,laserScanCPtr)){
                //ROS_INFO("HOLII");
                cout<<&laserScanCPtr<<endl;
            }
        }*/
        despl.setGoalReachedFlag(status);
        if (laserGot && trajReceived && !status)
        {

            despl.navigate(&nextPoint, &globalGoal);

            if (despl.finished())
                status = true;
        }
        
        loop_rate.sleep();
    }
    

    return 0;
}
void laserScanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
    laserScanCPtr = scan;
    laserScan = *scan;
    laserGot = true;
}
void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
    trajReceived = true;

    int i = 0;

    if (trj->points.size() > 1)
        i = 1;

    nextPoint = trj->points[i];
}
//Used to calculate distance from base_link to global goal
void globalGoalCallback(const geometry_msgs::PoseStampedConstPtr &globGoal_)
{
    status = false;
    globalGoal = *globGoal_;
}