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
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <people_msgs/People.h>
#include <navigator/securityMargin.hpp>

using namespace std;


namespace Navigators
{


typedef geometry_msgs::PoseStamped PoseStamp;
typedef visualization_msgs::Marker RVizMarker;

class Displacement
{
public:

  /**
   * Default constructor:
   * @param *NodeHandle: Pointer to node handle use to publish to topics 
   * @param *margin: Pointer to SecurityMargin object 
   * @param *tfBuffer: Pointer to tf2 buffer 
  **/
  Displacement(ros::NodeHandle *n, SecurityMargin *margin_, tf2_ros::Buffer *tfBuffer_);
  
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
  void trackedPersonCb(const people_msgs::People::ConstPtr &pl);
  void trajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj);
  void globalGoalCb(const geometry_msgs::PoseStampedConstPtr &globGoal_);
  void impossibleMoveCb(const std_msgs::Bool::ConstPtr &msg);
  void occLocalGoalCb(const std_msgs::Bool::ConstPtr &msg);
private:

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
    return angle/180*M_PI;
  }
  inline float rad2d(float angle){
    return angle/M_PI*180;
  }

  void printPeople();
  void computeDistanceToPeople();

  bool someoneInFront();
  bool someoneInFront(float vx, float vy);
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
  void moveHolonTest(double finalYaw);
  /**
   * No-Holonomic displacement function, called by the navigate function
  **/
  void moveNonHolon();
  void moveNonHolonTest();
  /**
   * Auxiliar function to get the yaw in degress from a quaternion
   * @param quat: quaternion to get the yaw from
  **/
  float getYawFromQuat(geometry_msgs::Quaternion quat);

 
  /**
   * Refresh new dynamically reconfigure set params
   * 
  **/
  void refreshParams();
  /**
   * Exponential speed calculator
   * @max: speed at inf
   * @exp_const: the decay constant
   * @var: the variable to be function of
   * v=max*(1-exp(-exp_cons*var))
  **/
  float getVel(float max, float exp_const, float var);



  /**    Variables    **/
  string world_frame, robot_frame;
  bool holonomic;//1 o 0(true or false)
  bool finalOrientationOk, homePublished, trajReceived;//Control flags
  bool do_navigate;
  double Vx, Vy, Wz;//Velocity variables
  double dist2GlobalGoal, dist2NextPoint;//Distances variables 

  float angle2NextPoint, angle2GlobalGoal;//Angles variables
  float angleMargin, distMargin;//Margins 
  float angularMaxSpeed, linearMaxSpeed; //Top speeds 

  //Custom speed testing parameters
  float v,a,b,old_b;
  float aproachDistance;
  float startOrientateDist;

  ros::NodeHandle *nh; //Pointer to the node node handle

  geometry_msgs::Twist vel; //The twist message that will be published
  geometry_msgs::PoseStamped globalGoal, globalGoalPose; //global goal in base_link and map frame.It would be nice to rename

  SecurityMargin *margin;//Pointer to the security margin object used to evaluate if it can move

  tf2_ros::Buffer *tfBuffer;//Pointer to the tfBuffer created in the node

  std_msgs::Bool muvingState, goalReached, localGoalOcc, manoeuvreStatus; //Flags that will be published 
  
  
  //std_msgs::UInt8MultiArray red, green, blue, white; //Not used right know, maybe will be used to signalize the status of the robot with the leds

  ros::Publisher twist_pub, muving_state_pub, goal_reached_pub, goal_pub, leds_pub,aproach_maneouvre_pub,dist2goal_pub; //Ros publishers 
  std_msgs::Float32 dist2goal;
  people_msgs::People peopl;
  std::map<string, pair<float, float>> dist2people; 
  std::pair<string, pair<float,float>> closest;
  float alpha;

  trajectory_msgs::MultiDOFJointTrajectoryPoint nextPoint; //next point of the trajetory received
 
  float traj_timeout;
  float delta;

  std_msgs::Bool possible_to_move;
  ros::Time start;
  ros::Duration d;
  float secs;

  bool aproxComplete,tr0_catch,factoryAproach, outOfTime;
  geometry_msgs::TransformStamped tr0,tr1;
  geometry_msgs::Vector3 dlt;
  int n_goals;
  bool traj_n_received;
};

} /*  namespace Navigators  */

#endif /* DISPLACEMENT_H_ */