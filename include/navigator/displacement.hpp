/*
* Displacement Class: For holonomic and no holonomic robots
* Rafael Rey Arcenegui 2019, UPO
*/

#ifndef DISPLACEMENT_H_
#define DISPLACEMENT_H_

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <limits>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <people_msgs/People.h>
#include <navigator/securityMargin.hpp>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <upo_actions/NavigateAction.h>
#include <upo_actions/RotationInPlaceAction.h>

#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/server.h>
#include <arco_path_tracker/navConfig.h>

using namespace std;

namespace Navigators
{

typedef geometry_msgs::PoseStamped PoseStamp;
typedef visualization_msgs::Marker RVizMarker;
typedef actionlib::SimpleActionServer<upo_actions::NavigateAction> NavigateServer;
typedef actionlib::SimpleActionServer<upo_actions::RotationInPlaceAction> RotationInPlaceServer;
class Displacement
{
public:
  /**
   * Default constructor:
   * @param *NodeHandle: Pointer to node handle use to publish to topics 
   * @param *margin: Pointer to SecurityMargin object 
   * @param *tfBuffer: Pointer to tf2 buffer 
  **/
  Displacement(tf2_ros::Buffer *tfBuffer_);

  /**
   * Make a aproximation manoeuvre smoothly
   * @param *pose: Pointer to pose in map frame to get to
   * @param isGoal: If true publish to the goal reached topic and change the flag to true once manouevre achieved
   * @param isHome: True when used with the goHomeLab function, it should approximate backwards
  **/
  void aproximateTo(geometry_msgs::PoseStamped *pose, bool isGoal, bool isHome);

  /**
   * Main function, it takes the next pose to go and pass it
   * to the holonomic/no-holonomic navigastion functions
   * @param isHome: To use when goHomeLab works
  **/
  void navigate();

  /**
   * Return the value of goalReached flag
   * @return goalReached.data
  **/
  bool hasFinished();

  /**
   * Set the flag goalReached to the status_ given
   * @param status_: If true, it changes goalReached to true and also publish zero velocity
  **/
  void setGoalReachedFlag(bool status_);

  /**
   * Rotation in place functions. You can personalize what it does with the parameters
   * @param q/yaw: Quaternion or yaw to to the rotation 
   * @param goal: If true it changes the goal reached flag etc
   * @parm pub: If false, the function only gives values to Wz and do not publish it
   * @param speed: The top speed in radians we want 
   * @param angleMargin_: The margin in degress to get the orientation. 10-15º is nice
  **/
  void setRobotOrientation(geometry_msgs::Quaternion q, bool goal, bool pub, float speed, float angleMargin_);
  void setRobotOrientation(float finalYaw, bool goal, bool pub, float speed, float angleMargin_);

  /**
   * Callbacks to get the needed info 
  **/
  void trajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj);
  void dynReconfCb(arco_path_tracker::navConfig &config, uint32_t level);

private:
  bool activateBackwardSrv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep);
  void laser1Callback(const sensor_msgs::LaserScanConstPtr &scan);
  void laser2Callback(const sensor_msgs::LaserScanConstPtr &scan);
  bool validateRotInPlace();
  bool rotateToRefresh();
  /**
   * Functions use to transform mainly between map and base_link frames
   * @param originalPose/point: The position we want to transform
   * @param from: frame of the original pose/point
   * @param to: frame in which we want the pose
   * @return: pose stamped in the desired frame
  **/
  PoseStamp transformPose(PoseStamp originalPose, std::string from, std::string to);
  PoseStamp transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to);

  /**
   * Functions to help to calculate distances
   * You can use pure coordinates, distance from two poses 
   * or finally the distance from one pose to the origin
   * @return distance 
  **/
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
  /**
   *  Aux function to get an angle in radians and viceversa
   *  @angle to convert to radians
  **/
  inline float d2rad(float angle)
  {
    return angle / 180 * M_PI;
  }
  inline float rad2d(float angle)
  {
    return angle / M_PI * 180;
  }
  /**
   * As its name says, it publishes the Vx,Vy and Wz
   * Also publishes to muving state topic as true if any of the velocity componentes are differents from zero
   * and the goal reached topic. This two topics are mainly used by the ros_bridge to communicate with camunda
  **/
  void publishCmdVel();

  /**
   * It puts Vx, Vy and Wz to zero and publish the twist messages,
   * also publishes the muving state as false and the value of goal reached value at that moment 
  **/
  void publishZeroVelocity();

  /**
   * Holonomic displacement function, called by the navigate function
   * @param finalYaw: If you want to force a final yaw
  **/
  void moveHolon(double finalYaw);
  /**
   * No-Holonomic displacement function, called by the navigate function
  **/
  void moveNonHolon();
  /**
   * Auxiliar function to get the yaw in degress from a quaternion
   * @param quat: quaternion to get the yaw from
  **/
  float getYawFromQuat(geometry_msgs::Quaternion quat);

  /**
   * Exponential speed calculator
   * @max: speed at inf
   * @exp_const: the decay constant
   * @var: the variable to be function of
   * v=max*(1-exp(-exp_cons*var))
  **/
  inline float getVel(float max, float exp_const, float var)
  {
    return max * (1 - exp(-exp_const * fabs(var))) * var / fabs(var);
  }

  void navGoalCb();
  void navPreemptCb();
  void rotGoalCb();
  void rotPreemptCb();

  //?
  sensor_msgs::LaserScanConstPtr scanL, scanR;
  bool scanRGot, scanLGot;
  bool backwards;
  ros::Time timeout_backwards;
  bool debug;
  //?
  /**    Variables    **/
  string world_frame, robot_frame;

  bool holonomic;    //1 o 0(true or false)
  bool trajReceived; //Control flags
  bool do_navigate;
  bool outOfTime;
  bool recoveryRotation;

  float rec_rot, rot_start;

  float Vx, Vy, Wz;                        //Velocity variables
  float dist2GlobalGoal, dist2NextPoint;   //Distances variables
  float angle2NextPoint, angle2GlobalGoal; //Angles variables
  float angleMargin, distMargin;           //Margins
  float angularMaxSpeed, linearMaxSpeed;   //Top speeds

  //Custom speed testing parameters
  float v, a, b;
  float startOrientateDist;

  visualization_msgs::Marker speed, rot_speed;

  ros::NodeHandlePtr nh; //Pointer to the node node handle

  ros::Publisher twist_pub, moving_state_pub, dist2goal_pub, speed_marker_pub, approach_man_pub, rot_recovery_status_pub; //Ros publishers
  ros::Subscriber laser1_sub, laser2_sub;
  ros::ServiceServer backwardsServer;
  ros::Time start;
  ros::Duration d;

  geometry_msgs::Twist vel;                              //The twist message that will be published
  geometry_msgs::PoseStamped globalGoal, globalGoalPose; //global goal in base_link and map frame.It would be nice to rename

  PoseStamp start_pose;      //Used in recov rot
  tf2_ros::Buffer *tfBuffer; //Pointer to the tfBuffer created in the node

  std_msgs::Bool movingState, goalReached, rot; //Flags that will be published
  std_msgs::Float32 dist2goal;

  people_msgs::People peopl;

  trajectory_msgs::MultiDOFJointTrajectoryPoint nextPoint; //next point of the trajetory received

  SecurityMargin margin;

  bool timeout = false;
  bool navigationPaused = false;
  ros::Time last_trj_stamp, time_count;

  std::unique_ptr<NavigateServer> navigate_server_ptr;
  std::unique_ptr<RotationInPlaceServer> rot_server_ptr;

  upo_actions::NavigateResult navigate_result;
  upo_actions::NavigateActionFeedback navigate_fb;
  upo_actions::NavigateGoalConstPtr navigate_goal;
  upo_actions::RotationInPlaceResult rot_result;
  upo_actions::RotationInPlaceGoalConstPtr rot_inplace;
  upo_actions::RotationInPlaceActionFeedback rotation_fb;
};

} /*  namespace Navigators  */

#endif /* DISPLACEMENT_H_ */
