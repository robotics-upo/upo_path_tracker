/*
* Navigator Class: For holonomic and no holonomic robots
* Rafael Rey Arcenegui 2019, UPO
*/

#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace std;

//Defines aqui
//Aqui clases
namespace Navigators
{

//Typedefs aqui
typedef geometry_msgs::PoseStamped PoseStamp;
typedef visualization_msgs::MarkerArray RVizMarkerArray;
typedef visualization_msgs::Marker RVizMarker;

class SecurityMargin
{
public:
  SecurityMargin(ros::NodeHandle *n);

  SecurityMargin(bool onlyFront_, int laserArrayMsgLen_, float f_, float innerSecDist_, float extSecDist_, int laserSecurityAngle_, ros::NodeHandle *n);

  void setParams(bool onlyFront_, int laserArrayMsgLen_, float f_, float innerSecDist_, float extSecDist_, int laserSecurityAngle_, ros::NodeHandle *n);
  
  void buildArrays();

  void publishRvizMarkers();
  //Check if there is something inside the security area
  bool checkObstacles(bool whichOne, sensor_msgs::LaserScan *scan);

  bool dangerAreaFree();
  bool securityAreaFree();

private:
  bool onlyFront; //TODO: implementar zonas de seguridad asimetricas delante y detras
  bool paramsConfigured;
  //Dangerous area is the area inside the inner ellipse,
  //securityArea is the area between extern and inner ellipse
  bool isInsideDangerousArea, securityAreaOccup;

  int laserArrayMsgLen; //721 for hokuyo lasers
  //f es la relacion entre el semieje menor y el semieje mayor de la elipse de seguridad,
  //es necesario en el caso de robots no simetricos como el ARCO y depende de su geometria
  //Mas redondo menor f y viceversa
  float f;
  float innerSecDist;
  float extSecDist;
  int laserSecurityAngle;

  RVizMarker markerInt, markerExt;

  ros::Publisher marker_pub;
  ros::NodeHandle *nh;

  vector<float> secArray, secArrayExt;
};

class Displacement
{
public:
  Displacement(ros::NodeHandle *n, Navigators::SecurityMargin *margin_, sensor_msgs::LaserScan *laserScan_,tf2_ros::Buffer *tfBuffer_);

  void navigate(trajectory_msgs::MultiDOFJointTrajectoryPoint *nextPoint, geometry_msgs::PoseStamped *globalGoalMapFrame);

  bool finished();

private:
  PoseStamp transformPose(PoseStamp originalPose, std::string from, std::string to);

  //Distance calculation formulas
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

  //Aux functions
  float getYawFromQuat(geometry_msgs::Quaternion quat);

  ros::NodeHandle *nh;
  ros::Publisher twist_pub, muving_state_pub, goal_reached_pub;
  

  bool holonomic;
  bool finalOrientationOk;

  double Vx, Vy, Wz;
  double dist2GlobalGoal;

  float angleMargin, distMargin;
  float angularMaxSpeed, linearMaxSpeed;

  geometry_msgs::Twist vel;
  Navigators::SecurityMargin *margin;
  sensor_msgs::LaserScan *laserScan;
  PoseStamp globalGoalPose;
  tf2_ros::Buffer *tfBuffer;
  std_msgs::Bool muvingState, goalReached;

};

} // namespace Navigators

#endif