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

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
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
typedef visualization_msgs::MarkerArray RVizMarkerArray;
typedef visualization_msgs::Marker RVizMarker;

class Displacement
{
public:
  //Default constructor: you have to pass it the pointers to the objects neededs
  //TODO: use ros ptr messages or const ptr
  Displacement(ros::NodeHandle *n, SecurityMargin *margin_, sensor_msgs::LaserScan *laserScan_,tf2_ros::Buffer *tfBuffer_, bool holon_);
  //Navigate function rights now is only no - holonmic
  void aproximateTo(geometry_msgs::PoseStamped *pose);
  
  //holonomic navigation
  void moveHolon(double theta, double dist2GlobalGoal_);
  //Non-holonomic navigation(orientate the robot toward the next point and go ahead)
  void moveNonHolon(double angle2NextPoint_, double dist2GlobalGoal_);
  void navigate(trajectory_msgs::MultiDOFJointTrajectoryPoint *nextPoint, geometry_msgs::PoseStamped *globalGoalMapFrame);
  //get goal reached message flag value

  bool finished();

  void setGoalReachedFlag(bool status_);
  //Functions to do a rotation in place, you can give it a quaternion or a yaw
  void setRobotOrientation(geometry_msgs::Quaternion q, bool goal, bool pub, float speed, float angleMargin_);
  void setRobotOrientation(float finalYaw, bool goal, bool pub, float speed, float angleMargin_);

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
  //Publish values to cmd vel topic
  //TODO: make the command vel topic a parameter
  void publishCmdVel();

  //The yaw is returned in degrees
  float getYawFromQuat(geometry_msgs::Quaternion quat);

  ros::NodeHandle *nh;
  ros::Publisher twist_pub, muving_state_pub, goal_reached_pub;
  

  bool holonomic;
  bool finalOrientationOk;

  double Vx, Vy, Wz;
  double dist2GlobalGoal, dist2NextPoint;
  float angle2NextPoint, angle2GlobalGoal;

  float angleMargin, distMargin;
  float angularMaxSpeed, linearMaxSpeed;

  geometry_msgs::Twist vel;
  SecurityMargin *margin;
  sensor_msgs::LaserScan *laserScan;
  PoseStamp globalGoalPose;
  tf2_ros::Buffer *tfBuffer;
  std_msgs::Bool muvingState, goalReached;

};

} // namespace Navigators

#endif