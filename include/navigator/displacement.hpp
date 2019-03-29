/*
* Displacement Class: For holonomic and no holonomic robots
* Rafael Rey Arcenegui 2019, UPO
*/

#ifndef DISPLACEMENT_H_
#define DISPLACEMENT_H_

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <navigator/securityMargin.hpp>

using namespace std;

//Defines aqui
//Aqui clases
namespace Navigators
{

//Typedefs aqui
typedef geometry_msgs::PoseStamped PoseStamp;
typedef visualization_msgs::Marker RVizMarker;

class Displacement
{
public:
  //Default constructor: you have to pass it the pointers to the objects neededs
  Displacement(ros::NodeHandle *n, SecurityMargin *margin_, tf2_ros::Buffer *tfBuffer_);
  //Navigate function rights now is only no - holonmic
  void goHomeLab();

  void aproximateTo(geometry_msgs::PoseStamped *pose, bool isGoal, bool isHome);

  void navigate(bool isHome);
  //void navigate(trajectory_msgs::MultiDOFJointTrajectoryPoint *nextPoint, PoseStamp *globalGoalMapFrame, bool isHome);
  //get goal reached message flag value

  bool hasFinished();

  void setGoalReachedFlag(bool status_);
  //Functions to do a rotation in place, you can give it a quaternion or a yaw
  void setRobotOrientation(geometry_msgs::Quaternion q, bool goal, bool pub, float speed, float angleMargin_);
  void setRobotOrientation(float finalYaw, bool goal, bool pub, float speed, float angleMargin_);

  void trajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj);
  void globalGoalCb(const geometry_msgs::PoseStampedConstPtr &globGoal_);

private:
  //Transform pose stamped between frames
  PoseStamp transformPose(PoseStamp originalPose, std::string from, std::string to);
  PoseStamp transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to);
  //Distance calculation formulas for differents inputs
  inline double euclideanDistance(double x0, double y0, double x, double y)
  {
    return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
  }
  inline double euclideanDistance(PoseStamp pose1, PoseStamp pose2)
  {
    return euclideanDistance(pose1.pose.position.x, pose1.pose.position.y, pose2.pose.position.x, pose2.pose.position.y);
  }
  inline double euclideanDistance(PoseStamp next)
  {
    return euclideanDistance(0, 0, next.pose.position.x, next.pose.position.y);
  }

  void publishCmdVel();
  void publishZeroVelocity();
  //holonomic navigation
  void moveHolon(double theta, double dist2GlobalGoal_, double finalYaw);
  //Non-holonomic navigation(orientate the robot toward the next point and go ahead)
  void moveNonHolon(double angle2NextPoint_, double dist2GlobalGoal_);
  //The yaw is returned in degrees
  float getYawFromQuat(geometry_msgs::Quaternion quat);

  int holonomic;
  bool finalOrientationOk, homePublished, trajReceived;

  double Vx, Vy, Wz;
  double dist2GlobalGoal, dist2NextPoint;

  float angle2NextPoint, angle2GlobalGoal;
  float angleMargin, distMargin;
  float angularMaxSpeed, linearMaxSpeedX, linearMaxSpeedY;

  ros::NodeHandle *nh;

  geometry_msgs::Twist vel;
  geometry_msgs::PoseStamped globalGoal, globalGoalPose;

  SecurityMargin *margin;

  tf2_ros::Buffer *tfBuffer;

  std_msgs::Bool muvingState, goalReached;
  std_msgs::UInt8MultiArray red, green, blue, white;

  ros::Publisher twist_pub, muving_state_pub, goal_reached_pub, goal_pub, leds_pub;

  trajectory_msgs::MultiDOFJointTrajectoryPoint nextPoint;
};

} // namespace Navigators

#endif