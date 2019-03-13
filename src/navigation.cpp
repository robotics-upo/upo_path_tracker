#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <math.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

//#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include <LinearMath/btMatrix3x3.h>
//Comment or un comment to get debug info while running
//#define DEBUG

double xMove, yMove, zRot;
geometry_msgs::Pose myPose;

geometry_msgs::Pose goalPose;
bool goalReceived = false;
bool localGoalReached = false;
double linear_max_speed = 0.5;
double angular_max_speed = 0.6;
//tf::TransformListener *tf_listener;
geometry_msgs::PoseStamped msgGoalPoseStamped;
geometry_msgs::PoseStamped goalPoseStamped;

//visualization_msgs::MarkerArray markerArray;
trajectory_msgs::MultiDOFJointTrajectory trajectory;
tf2_ros::Buffer tfBuffer;

double lastAngle = 1;
int angleOK = 0;
int positionOK = 0;
int goalCount = 10;
int trackLength = 0;

void TrackCallBack(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
    trajectory = *trj;
    goalReceived = true;
    angleOK = 0;
    positionOK = 0;
    lastAngle = 1;
    goalCount = 10;
    trackLength = trajectory.points.size();
#ifdef DEBUG
    ROS_INFO("PATH TRACKER:LOCAL TRAJECTORY GOT");
#endif
}

int getSign(double x)
{
    if (x > 0)
        return 1;
    if (x < 0)
        return -1;
    return 0;
}

geometry_msgs::Pose transformPose(geometry_msgs::Pose originalPose, std::string originalTF, std::string nextTF)
{
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tfBuffer.lookupTransform(nextTF, originalTF, ros::Time(0));
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

    tf2::doTransform(originalPoseStamped, nextPoseStamped, transformStamped);
    return nextPoseStamped.pose;
}

class Navigation
{
    geometry_msgs::Twist move;

  public:
    void navigate();
    Navigation(ros::Publisher twist_publisher);

  private:
    ros::Publisher pub_twist;
    double euclideanDistance(double x0, double y0, double x, double y)
    {

        return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
    }
    double euclideanDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
    {

        return sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2));
    }
    void publish(double xVel, double yVel, double angularVel);
};
Navigation::Navigation(ros::Publisher twist_publisher)
{
    pub_twist = twist_publisher;
}
void Navigation::navigate()
{

    double angleToNextpoint = atan2(goalPose.position.y /* - myPose.position.y*/, goalPose.position.x /* - myPose.position.x*/) /*- myAngle*/;
#ifdef DEBUG
    ROS_INFO("goal pose: [ %f , %f, %f ]", goalPose.position.x, goalPose.position.y, angleToNextpoint);
#endif
    if (angleToNextpoint > M_PI)
    {
        angleToNextpoint -= 2 * M_PI;
    }

    double linearVelX = 0;
    double linearVelY = 0;

    double angularVel = 0;
    double euclideanDist = euclideanDistance(myPose, goalPose);
    if (euclideanDist < 0.2)

    {
        goalPose.position.x = trajectory.points[1].transforms[0].translation.x; 
        goalPose.position.y = trajectory.points[1].transforms[0].translation.y;
        goalPose.orientation = trajectory.points[1].transforms[0].rotation;
        goalPose = transformPose(goalPose, "map", "base_link");
    }
    if (euclideanDist < 1.5)
    {
        double diffAngle = angleToNextpoint; // - myAngle;

        if (diffAngle > M_PI)
        {
            diffAngle -= 2 * M_PI;
        }
        else if (diffAngle < -M_PI)
        {
            diffAngle += 2 * M_PI;
        }
        if (lastAngle == 1)
        {
            lastAngle = diffAngle;
        }
        if (euclideanDist < 0.2) 
        {
            if (trackLength <= 1) // check if there is only a goal left before checking if the final goal was reached
    
            {
                if (positionOK < goalCount)
                {
                    linearVelX = linear_max_speed * cos(angleToNextpoint) / 4;
                    linearVelY = linear_max_speed * sin(angleToNextpoint) / 4;
                }
                if (euclideanDist < 0.05)
                {
                    if (++positionOK == goalCount)
                    {
                        ROS_INFO("POSITION OK");
                        localGoalReached = true;
                    }
                }
                else
                {
                    positionOK = 0;
                }
            }
        }
        else
        {
            linearVelX = linear_max_speed * cos(angleToNextpoint);
            linearVelY = linear_max_speed * sin(angleToNextpoint);
        }
        if (angleOK < goalCount)
        {
            if (fabs(diffAngle) > 0.5)
            {
                angularVel = angular_max_speed * getSign(diffAngle);
            }
            else if (fabs(diffAngle) > 0.005)
            {
                if (fabs(diffAngle) > fabs(lastAngle))
                {
                    angularVel = diffAngle / 2;
                }
                else
                {
                    angularVel = diffAngle;
                }
                angleOK = 0;
            }
            else
            {
#ifdef DEBUG
                ROS_INFO("robot pos: x %f, y %f, z %f ", myPose.position.x, myPose.position.y, myAngle);
                ROS_INFO("goal  pos: x %f, y %f, z %f ", goalPose.position.x, goalPose.position.y, goalAngle);
                ROS_INFO("diffangle: %f", diffAngle);
#endif
            if (trackLength <= 1) // check if there is only a goal left before checking if the final goal was reached
                {

                    if (++angleOK == goalCount)
                    {
#ifdef DEBUG
                        ROS_INFO("ANGLE OK");
#endif
                    }
                }
            }
        }
    }
    else
    {

        angularVel = sin(angleToNextpoint / 2);
        if (fabs(angularVel) > angular_max_speed)
        {
            angularVel = angular_max_speed * angularVel / fabs(angularVel);
        }
        if (fabs(angleToNextpoint) < 1)
        {
            linearVelX = linear_max_speed - linear_max_speed * (fabs(angleToNextpoint) / M_PI);
        }
    }
    publish(linearVelX, linearVelY, angularVel);
    if (positionOK == goalCount && angleOK == goalCount)
    {
#ifdef DEBUG
        ROS_INFO("Goal Reached");
        ROS_INFO("Robot pos: x %f, y %f, z %f ", myPose.position.x, myPose.position.y, myAngle);
        ROS_INFO("Goal  pos: x %f, y %f, z %f ", goalPose.position.x, goalPose.position.y, goalAngle);
#endif
    }
    // ROS_INFO("Angular: %f, linearVelX: %f, linearVelY: %f", angularVel, linearVelX, linearVelY);
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
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arco_path_tracker");

    std_msgs::Bool message;
    message.data = false;
    myPose.position.x = 0;
    myPose.position.y = 0;
    myPose.orientation.w = 1;
    myPose.orientation.z = 0;
    myPose.orientation.x = 0;
    myPose.orientation.y = 0;

    ros::NodeHandle n;
    ros::Subscriber path_sub = n.subscribe("/trajectory_tracker/local_input_trajectory", 1000, TrackCallBack);
    ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/idmind_motors/twist/", 1000);
    ros::Publisher local_goal_reach_pub = n.advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_reached", 10);
    local_goal_reach_pub.publish(message);

    ros::ServiceClient odom_client = n.serviceClient<std_srvs::Trigger>("/idmind_odometry/reset_position");

    tf2_ros::TransformListener tfListener(tfBuffer);

    Navigation navigation(pub_twist);

    n.getParam("linear_max_speed", linear_max_speed);
    n.getParam("angular_max_speed", angular_max_speed);

    bool succes = false;
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();

        if (goalReceived && !(angleOK == goalCount && positionOK == goalCount))
        {

            if (!localGoalReached && message.data)
            {
                message.data = false;
                local_goal_reach_pub.publish(message);
            }
         
                while (true)
                {
                    try
                    {
                        goalPose.position.x = trajectory.points[0].transforms[0].translation.x; 
                        goalPose.position.y = trajectory.points[0].transforms[0].translation.y;
                        goalPose.orientation = trajectory.points[0].transforms[0].rotation;
                        goalPose = transformPose(goalPose, "map", "base_link");

                        succes = true;
                        lastAngle = 1;
                        break;
                    }
                    catch (const std::exception &ex)
                    {
                        succes = false;
                        continue;
                    }
                }
#ifdef DEBUG
                ROS_INFO("TRAJ TRACKER : GOAL POSE [%.2f, %.2f]", goalPose.position.x, goalPose.position.y);
#endif
                if (succes)
                {
                    navigation.navigate();
                    if (localGoalReached)
                    {
                        message.data = true;
                        local_goal_reach_pub.publish(message);
                    }
                }
            
        }

        loop_rate.sleep();
    }

    return 0;
}
