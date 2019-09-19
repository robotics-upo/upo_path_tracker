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
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

#define PRINTF_REGULAR  "\x1B[0m"
#define PRINTF_RED  	"\x1B[31m"
#define PRINTF_GREEN  	"\x1B[32m"
#define PRINTF_YELLOW  	"\x1B[33m"
#define PRINTF_BLUE  	"\x1B[34m"
#define PRINTF_MAGENTA  "\x1B[35m"
#define PRINTF_CYAN  	"\x1B[36m"
#define PRINTF_WHITE	"\x1B[37m"

using namespace std;

class SecurityMargin
{

public:
  /**
   * Default constructor: It calls the setParams function and build the arrays of the 
   * security margin If pubmarkers is true, it also publish the markers of the security arrays
   * @param *n: pointer to a node handle
   * @param *tfBuffer_ : pointer to tfbuffer used by a tf listener
  **/
  SecurityMargin();

  SecurityMargin(ros::NodeHandle *n);

  /**
   * This function plays the game. If something enter inside the inner margin(frontal or back) 
   * it will be publishing false until the object or person has gone away further than the outer margin
   * If there is something inside the outer margin but out of the inner, it will return true
  **/
  bool canIMove();
  void setMode(int mode);
  /**
   * Callbacks to get the directly the laser information
  **/
  void laser1Callback(const sensor_msgs::LaserScanConstPtr &scan);
  void goalReachedCb(const std_msgs::Bool::ConstPtr &msg);
  bool switchCtrlSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep);

    /**
   * Put to default the parameters values if any on the parameter server
   * It also put the flags to their initial values and if pub markers is set 
   * to true it initilize the markers messages
   * @param *n: pointer to a node handle
  **/
  void setParams(ros::NodeHandle *n);

private:
  
  //void buildSelected();
  /**
   * This function build the four security margins arrays and also 
   * the marker of each one if pub markers is set to true
   * @param ext: If you want to build the inner or the outer margin
  **/
  //void buildArraysSquare2(bool ext);
  /**
   * Used at the demo and other experiments
   * 
   * 
  **/
  void buildElliptic();
  /**
   * Old array builder for the old platform
   * 
   * 
  **/
  //void buildArrays();
  /**
   * If only front is set to true, it only publishes the front security margin markers 
  **/
  void publishRvizMarkers();

  /**
   * The main function that compares the arrays with the lasers messages data+
   * @param whickOne: 1 to check the outer margins and 0 for the inner margin
   * @return: true if something inside the selected area
  **/
  bool checkObstacles();



  /**     Variables      **/
  
  bool laserGot,paramsConfigured;
  int status, laserMsgLen, cnt,cnt2;
  float f,innerSecDistFront, extSecDistFront;
  
  //Frames and topics parameters
  string base_link_frame;

  vector<float> secArrayFr, secArrayExtFr;

  ros::NodeHandle *nh;
  ros::Publisher marker_int_pub, marker_ext_pub, stop_pub;
  ros::ServiceServer enableManualSrv;
  ros::ServiceClient stopMotorsSrv;
  ros::Subscriber laser1_sub,aproach_man_sub, dist2goal_sub;
  
  sensor_msgs::LaserScanConstPtr laserCPtr;

  geometry_msgs::Vector3 dlt;
  geometry_msgs::TransformStamped tr0,tr1;

  std_msgs::ColorRGBA red, green,blue;//Colors for RViz markers
  std_msgs::Bool stop_msg,aproaching_status, goal_reached;
  //Markers for RViz, *back markers are only used in the old arco (build array function with only_front paramter to true)
  visualization_msgs::Marker markerIntFr, markerExtFr;

};

#endif
