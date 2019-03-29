/*
* Security Margin Class: Let you create security margin objects to use with the displacement class
* Rafael Rey Arcenegui 2019, UPO
*/

#ifndef SECURITYMARGIN_H_
#define SECURITYMARGIN_H_

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
//Defines
#define HOKUYO 721

class SecurityMargin
{
  typedef geometry_msgs::PoseStamped PoseStamp;
  typedef visualization_msgs::Marker RVizMarker;

public:
  //Default constructor: it set params to defaults values if nothing on the param server
  SecurityMargin(ros::NodeHandle *n);

  //It passes the params to the class variables
  void setParams(ros::NodeHandle *n);
  //Publish the rviz markers to visualize the security margin in RViz
  void publishRvizMarkers();
  
  //Check if there is something inside the security area
  bool canIMove();

  //get values of booleans private variables
  bool dangerAreaFree();
  bool securityAreaFree();
  //Lasers Callbacks
  void laser1Callback(const sensor_msgs::LaserScanConstPtr &scan);
  void laser2Callback(const sensor_msgs::LaserScanConstPtr &scan);

private:
  //Builds the  security arrays and markers
  void buildArrays();

  bool checkObstacles(bool whichOne);

  
  bool laser1Got, laser2Got, lasersGot;
  bool onlyFront;
  bool paramsConfigured;
  bool red1,red2;
  
  bool isInsideDangerousArea1, securityAreaOccup1;
  bool isInsideDangerousArea2, securityAreaOccup2;
  bool pubMarkers;

  int frontLaserArrayMsgLen, backLaserArrayMsgLen; //721 for hokuyo lasers
  int laserSecurityAngleFront, laserSecurityAngleBack;
  //f es la relacion entre el semieje menor y el semieje mayor de la elipse de seguridad,
  //es necesario en el caso de robots no simetricos como el ARCO y depende de su geometria
  //Mas redondo menor f y viceversa
  float f1, f2;
  float innerSecDistFront, extSecDistFront, innerSecDistBack, extSecDistBack;

  ros::NodeHandle *nh;
  
  sensor_msgs::LaserScanConstPtr laser1CPtr, laser2CPtr;
  
  RVizMarker markerIntFr, markerExtFr, markerIntBack, markerExtBack;
  
  ros::Publisher marker_fr_1_pub,marker_fr_2_pub,marker_rr_1_pub,marker_rr_2_pub;
  
  vector<float> secArrayFr, secArrayExtFr, secArrayBack, secArrayExtBack;
};

#endif