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



//Defines aqui
//Aqui clases


class SecurityMargin
{
    typedef geometry_msgs::PoseStamped PoseStamp;
    typedef visualization_msgs::MarkerArray RVizMarkerArray;
    typedef visualization_msgs::Marker RVizMarker;

public:
 

  //Default constructor: it set params to defaults values(see navigator.cpp)
  SecurityMargin(ros::NodeHandle *n);
  /** 
   * Constructor with parameters
   * 
   * @param onlyFront: to create only a security margin in the front, useful when the robot usually moves forward
   * the full security margin is not implemented yet
   * @param laserArrayMsgLen: the lenght of the .ranges array in the laser message dist2GlobalGoal
   * @param f: f=1 security margin circle like, >1 ellipse like
   * @param innerSecDist and extSecDist: The distances of the inner and exterior security margins
   * @param laserSecurityAngle: Set to 0 if you want to check the full range of the laser array, the security angle reduces it
   *   simmetrically. Ej: 10, the security margin will be checked between 10 and 170 degrees for a 180º laser(Hokuyo)
  */
  //It pases the params to the class variables
  void setParams(ros::NodeHandle *n);
  //Builds the two security arrays and markers 
  void buildArrays();
  //Publish the rviz markers to visualize the security margin in RViz
  void publishRvizMarkers();
  //Check if there is something inside the security area
  bool checkObstacles(bool whichOne, sensor_msgs::LaserScan *scan);
  bool canIMove(sensor_msgs::LaserScan *scan);
  //get values of booleans private variables
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
  float innerSecDist,extSecDist;
  int laserSecurityAngle;

  RVizMarker markerInt, markerExt;

  ros::Publisher marker_pub;
  ros::NodeHandle *nh;

  vector<float> secArray, secArrayExt;
};



#endif