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

class SecurityMargin
{
  typedef geometry_msgs::PoseStamped PoseStamp;
  typedef visualization_msgs::Marker RVizMarker;

public:
  /**
   * Default constructor: It calls the setParams function and build the arrays of the 
   * security margin If pubmarkers is true, it also publish the markers of the security arrays
   * @param *n: pointer to a node handle
  **/
  SecurityMargin(ros::NodeHandle *n);

  /**
   * This function plays the game. If something enter inside the inner margin(frontal or back) 
   * it will be publishing false until the object or person has gone away further than the outer margin
   * If there is something inside the outer margin but out of the inner, it will return true
  **/
  bool canIMove();

  /**
   * Get the dangerAreaFree status flag
  **/
  bool dangerAreaFree();

  /**
   * Get the security area status flag
  **/
  bool securityAreaFree();

  /**
   * Callbacks to get the directly the laser information
  **/
  void laser1Callback(const sensor_msgs::LaserScanConstPtr &scan);
  void laser2Callback(const sensor_msgs::LaserScanConstPtr &scan);

private:
  /**
   * This function build the four security margins arrays and also 
   * the marker of each one if pub markers is set to true
  **/
  void buildArraysSquare2(vector<float> *array, RVizMarker *marker, bool ext);
  /**
   * 
   * 
   * 
  **/
  void buildElliptic();
  /**
   * 
   * 
   * 
  **/
  void buildArrays();
  /**
   * If only front is set to true, it only publishes the front security margin markers 
  **/
  void publishRvizMarkers();

  /**
   * The main function that compares the arrays with the lasers messages data+
   * @param whickOne: 1 to check the outer margins and 0 for the inner margin
   * @return: true if something inside the selected area
  **/
  bool checkObstacles(bool extPerimeter);

  /**
   * Function used to refresh params when they are changed by dynamic reconfigure
   * 
  **/
  void refreshParams();

  /**
   * Put to default the parameters values if any on the parameter server
   * It also put the flags to their initial values and if pub markers is set 
   * to true it initilize the markers messages
   * @param *n: pointer to a node handle
  **/
  void setParams(ros::NodeHandle *n);

  /**     Variables      **/

  bool laser1Got, laser2Got, lasersGot; //lasers control flags
  bool onlyFront;                       //To use only frontal security margin
  bool paramsConfigured;
  bool red1, red2; //Flux control in checkObstacles function
  bool isInsideDangerousArea1, securityAreaOccup1; //Information about the security status
  bool isInsideDangerousArea2, securityAreaOccup2;
  bool pubMarkers;

  int secMode;
  int frontLaserArrayMsgLen, backLaserArrayMsgLen;     //721 for hokuyo lasers
  int laserSecurityAngleFront, laserSecurityAngleBack; //The angle to reduce the oppenning of the lasers data comparison
  //if it set 10-20, then the margin will check between 10 and 170º of the laser array angles

  //f es la relacion entre el semieje menor y el semieje mayor de la elipse de seguridad,
  //es necesario en el caso de robots no simetricos como el ARCO y depende de su geometria
  //Mas redondo menor f y viceversa
  float f1, f2;
  float innerSecDistFront, extSecDistFront, innerSecDistBack, extSecDistBack;
  float margin_y, margin_x, delta_d;

  string base_link_frame, front_laser_link_frame, back_laser_link_frame;

  vector<float> secArrayFr, secArrayExtFr, secArrayBack, secArrayExtBack;
  
  ros::NodeHandle *nh;
  ros::Publisher marker_fr_1_pub, marker_fr_2_pub, marker_rr_1_pub, marker_rr_2_pub, stop_pub;

  sensor_msgs::LaserScanConstPtr laser1CPtr, laser2CPtr;
  std_msgs::Bool stop_msg;
  std_msgs::ColorRGBA red, green;
  RVizMarker markerIntFr, markerExtFr, markerIntBack, markerExtBack;
  ros::Subscriber laser1_sub,laser2_sub;
  string laser1_topic,laser2_topic;
  
};

#endif
