/*
 * Security Margin Class: Let you create security margin objects to use with the displacement class
 * Rafael Rey Arcenegui 2019, UPO
 */

#ifndef SECURITYMARGIN_H_
#define SECURITYMARGIN_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#define PRINTF_REGULAR "\x1B[0m"
#define PRINTF_RED "\x1B[31m"
#define PRINTF_GREEN "\x1B[32m"
#define PRINTF_YELLOW "\x1B[33m"
#define PRINTF_BLUE "\x1B[34m"
#define PRINTF_MAGENTA "\x1B[35m"
#define PRINTF_CYAN "\x1B[36m"
#define PRINTF_WHITE "\x1B[37m"

using namespace std;

namespace Upo
{
namespace Navigation
{
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

  /**
   * This function plays the game. If something enter inside the inner margin(frontal or back)
   * it will be publishing false until the object or person has gone away further than the outer margin
   * If there is something inside the outer margin but out of the inner, it will return true
   **/
  bool canIMove();
  void setMode(int mode);

private:
  /**
   * Callbacks to get the directly the laser information
   **/
  void laser1Callback(const sensor_msgs::LaserScanConstPtr &scan);
  /**
   * Put to default the parameters values if any on the parameter server
   * It also put the flags to their initial values and if pub markers is set
   * to true it initilize the markers messages
   * @param *n: pointer to a node handle
   **/
  void setParams();

  /**
   * Used at the demo and other experiments
   *
   *
   **/
  void buildElliptic();
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
  ros::NodeHandlePtr nh;
  ros::Publisher marker_int_pub, marker_ext_pub, stop_pub;
  ros::Subscriber laser1_sub;

  std_msgs::ColorRGBA red, green, blue;  // Colors for RViz markers
  std_msgs::Bool stop_msg;
  // Markers for RViz, *back markers are only used in the old arco (build array function with only_front paramter to
  // true)
  visualization_msgs::Marker markerIntFr, markerExtFr;
  sensor_msgs::LaserScanConstPtr laserCPtr;

  bool laserGot;
  int status, laserMsgLen, cnt, cnt2;
  float f, innerSecDistFront, extSecDistFront;

  // Frames and topics parameters
  std::string base_link_frame;

  std::vector<float> secArrayFr, secArrayExtFr;
};
}  // namespace Navigation
}  // namespace Upo

#endif
