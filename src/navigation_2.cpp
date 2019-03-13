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

//#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
trajectory_msgs::MultiDOFJointTrajectory trajectory;

geometry_msgs::TransformStamped next_point;
geometry_msgs::PoseStamped globalGoal;

tf2_ros::Buffer tfBuffer;

std_msgs::Bool goalReached;
std_msgs::Bool muving_state;
sensor_msgs::LaserScan laserScanMsg;

bool laserGot = false;
bool trajReceived = false;
bool isInsideDangerArea = false;
bool securityAreaFree = true;
bool finalOrientationOk = false;
//Params
double linearMaxSpeed=0.5;
double angularMaxSpeed=4;
double distMargin = 0.35;            //Values by default
double angleMargin = 15;            //Values by default
double securityInnerDistance = 0.2; //By default
double securityExtDistance = 3 * securityInnerDistance;
double dist2GlobalPoint;
//Relacion entre el semieje menor y el semieje mayor de la elipse de seguridad. El semieje menor es la securityInnerDistance
double f = 1.6;

int arrayLaserLeng;
//The number is 10 is in degrees, the factor 721/180 is for conversion to array numbers
int laserSecurityAngle = ceil(15 * (721 / 180));
//Array to store security distance semi-ellipse
std::vector<float> secArray;
std::vector<float> secArrayExt;


void calculateSecurityArray()
{
    double x, y;
    //Cambiar el 721 por el arrayLaserLeng
    for (double i = 0; i < 721; i++)
    {
        x = securityInnerDistance * cos(i / 721 * M_PI - M_PI_2);
        y = f * securityInnerDistance * sin(i / 721 * M_PI - M_PI_2);
        secArray.push_back(sqrtf(x * x + y * y));

        x = securityExtDistance * cos(i / 721 * M_PI - M_PI_2);
        y = f * securityExtDistance * sin(i / 721 * M_PI - M_PI_2);
        secArrayExt.push_back(sqrtf(x * x + y * y));
    }
}
void laserScanCallback(const sensor_msgs::LaserScanConstPtr &scan)
{
    laserScanMsg = *scan;
    arrayLaserLeng = laserScanMsg.ranges.size();
    laserGot = true;
}
void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
    trajectory = *trj;
    trajReceived = true;
    goalReached.data = false;

    next_point.header.frame_id = "map";
    next_point.header.stamp = ros::Time(0);
    next_point.header.seq = rand();
    if (trajectory.points.size() > 1)
    {
        next_point.transform = trajectory.points[1].transforms[0];
        //ROS_WARN("Next point(inside traj callback): [%.2f,%.2f]", next_point.transform.translation.x, next_point.transform.translation.y);
    }
    else
    {
        next_point.transform = trajectory.points[0].transforms[0];
    }
}
//Used to calculate distance from base_link to global goal
void globalGoalCallback(const geometry_msgs::PoseStampedConstPtr &globgoal)
{
    globalGoal = *globgoal;
}
geometry_msgs::Pose transformPose(geometry_msgs::Pose originalPose, std::string originalTF, std::string nextTF)
{
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tfBuffer.lookupTransform(nextTF, originalTF, ros::Time(0));
        //ROS_WARN("TRANSFORM: [%.2f,%.2f] ", transformStamped.transform.translation.x, transformStamped.transform.translation.y);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("No transform %s", ex.what());
    }

    geometry_msgs::PoseStamped originalPoseStamped;
    geometry_msgs::PoseStamped nextPoseStamped;

    originalPoseStamped.pose = originalPose;
    originalPoseStamped.header.frame_id = originalTF;
    originalPoseStamped.header.stamp = ros::Time(0);

    nextPoseStamped.header.frame_id = nextTF;
    nextPoseStamped.header.stamp = ros::Time(0);
    //nextPoseStamped.pose.position = originalPose.position.x =
    tf2::doTransform(originalPoseStamped, nextPoseStamped, transformStamped);
    //ROS_WARN("NEXT POSE(INSIDE TRANSF): [%.2f,%.2f]", nextPoseStamped.pose.position.x, nextPoseStamped.pose.position.y);

    return nextPoseStamped.pose;
}

class Navigation
{
  public:
    void navigate();
    Navigation(ros::Publisher twist_publisher, ros::Publisher goal_reached, ros::Publisher muvin_state);

    void publish(double xVel, double yVel, double angularVel);

  private:
    ros::Publisher pub_twist;
    ros::Publisher goal_pub;
    ros::Publisher muv_pub;

    inline double euclideanDistance(double x0, double y0, double x, double y)
    {
        return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
    }
    inline double euclideanDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
    {
        return euclideanDistance(pose1.position.x, pose1.position.y, pose2.position.x, pose2.position.y);
    }
    inline double euclideanDistance(geometry_msgs::Pose next)
    {
        return euclideanDistance(0, 0, next.position.x, next.position.y);
    }
    float getYawFromQuat(geometry_msgs::Quaternion quat);
};

Navigation::Navigation(ros::Publisher twist_publisher, ros::Publisher goal_reached, ros::Publisher muvin_state)
{
    pub_twist = twist_publisher;
    goal_pub = goal_reached;
    muv_pub = muvin_state;
}
void Navigation::publish(double xVel, double yVel, double zVel)
{
    geometry_msgs::Twist vel;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = zVel;
    vel.linear.x = xVel;
    vel.linear.y = yVel;
    pub_twist.publish(vel);
    if(xVel != 0 || yVel != 0 || zVel != 0){
        muving_state.data  = true;
       
    }else{
        muving_state.data = false;
    }
    muv_pub.publish(muving_state);
    goal_pub.publish(goalReached);
}
bool checkObstacles(std::vector<float> *array, bool inner)
{
    bool ret = false;
    if (laserGot)
    {
        for (int i = laserSecurityAngle; i < arrayLaserLeng - laserSecurityAngle; i++)
        {
            if (laserScanMsg.ranges.at(i) < array->at(i) && laserScanMsg.ranges.at(i) > 0.02)
            {
                ret = true;
                ROS_ERROR("STOP! Something to near i %d", i);
                if (inner)
                {
                    isInsideDangerArea = true;
                    securityAreaFree = false;
                }

                break;
            }
        }
    }
    return ret;
}
void Navigation::navigate()
{

    double linearVelX = 0;
    double linearVelY = 0;
    double angularVel = 0;

    geometry_msgs::PoseStamped globalGoalPose;
    //The trajectory and the next_point transform is received in the map frame so we hace to transform it
    //to base_link frame to work with it
    geometry_msgs::Pose next_pointpose, next_pose;

    next_pointpose.position.x = next_point.transform.translation.x;
    next_pointpose.position.y = next_point.transform.translation.y;
    next_pointpose.orientation = next_point.transform.rotation;

    next_pose = transformPose(next_pointpose, "map", "base_link");
    double angle2Nextpoint = atan2(next_pose.position.y, next_pose.position.x);

    while (angle2Nextpoint > 2 * M_PI)
    {
        angle2Nextpoint -= 2 * M_PI;
    }
    if(angle2Nextpoint > M_PI ){
        angle2Nextpoint-=2*M_PI;
    }
    if(angle2Nextpoint < -M_PI){
        angle2Nextpoint += 2*M_PI;
    }
    double dist2NextPoint = euclideanDistance(next_pose);

    globalGoalPose.pose = transformPose(globalGoal.pose, "map", "base_link");
    dist2GlobalPoint = euclideanDistance(globalGoalPose.pose);
    double angle2GlobalGoal = Navigation::getYawFromQuat(globalGoalPose.pose.orientation);

    if (dist2GlobalPoint < distMargin && !goalReached.data)
    {
        
        angle2Nextpoint = atan2(globalGoalPose.pose.position.y, globalGoalPose.pose.position.x);
        while (fabs(angle2GlobalGoal) > angleMargin)
        {
            globalGoalPose.pose = transformPose(globalGoal.pose, "map", "base_link");
            angle2GlobalGoal = Navigation::getYawFromQuat(globalGoalPose.pose.orientation);
            angularVel = angularMaxSpeed;

            if (angle2GlobalGoal < 0)
                angularVel *= -1;
            if (fabs(angle2Nextpoint) < angleMargin * 2)
                angularVel /= 2;
           
            Navigation::publish(0, 0,angularVel);
        }
        finalOrientationOk = true;
        goalReached.data = true;
        Navigation::publish(0, 0, 0);
    }
    else if (!checkObstacles(&secArray, 1) && !goalReached.data)
    { //Rotation in place
        if (fabs(angle2Nextpoint) > (angleMargin+5) * M_PI / 180)
        {
            angularVel = angularMaxSpeed;
            if (angle2Nextpoint < 0)
                angularVel *= -1;
           
            Navigation::publish(0, 0, angularVel);
        }
        else
        {
            linearVelX = 0.4;
            angularVel = fabs(angle2Nextpoint) * 13 / (5 * M_PI);

            if (angle2Nextpoint < 0)
                angularVel *= -1;

            if (dist2GlobalPoint < distMargin * 2)
                linearVelX *= 0.4;
            Navigation::publish(linearVelX, linearVelY, angularVel);
        }
    }
    else if (checkObstacles(&secArray, 1))
    {
        Navigation::publish(0, 0, 0);
    }
}
float Navigation::getYawFromQuat(geometry_msgs::Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y / M_PI * 180;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "arco_path_tracker_2");
    goalReached.data = false;
    ros::NodeHandle n;

    ros::Subscriber path_sub = n.subscribe("/trajectory_tracker/local_input_trajectory", 1000, trajectoryCallback);
    ros::Subscriber laser_sub = n.subscribe("/scanFront", 1000, laserScanCallback);
    ros::Subscriber global_goal_sub = n.subscribe("/move_base_simple/goal", 10, globalGoalCallback);
    ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/idmind_motors/twist/", 1000);
    ros::Publisher local_goal_reach_pub = n.advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_reached", 10);
    ros::Publisher muving_pub = n.advertise<std_msgs::Bool>("/trajectory_tracker/muving_state",10);
    //Ros topic to communicate with local planner

    tf2_ros::TransformListener tfListener(tfBuffer);

    Navigation navigation(pub_twist, local_goal_reach_pub,muving_pub);

    ros::Rate loop_rate(40);

    calculateSecurityArray();

    while (ros::ok())
    {
        ros::spinOnce();

        if (isInsideDangerArea && !checkObstacles(&secArrayExt, 0))
        {
            isInsideDangerArea = false;
            securityAreaFree = true;
        }

        if (trajReceived && securityAreaFree && !goalReached.data)
        {
            //ROS_INFO("NAVIGATIN");
            navigation.navigate();
        }

        loop_rate.sleep();
    }

    return 0;
}