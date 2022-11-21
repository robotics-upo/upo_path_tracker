#ifndef NEW_ARCO_PATH_TRAJ_TRACKER_H_
#define NEW_ARCO_PATH_TRAJ_TRACKER_H_

// ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <upo_actions/NavigateAction.h>

#include <actionlib/server/simple_action_server.h>
#include <visualization_msgs/Marker.h>

#include <std_srvs/Trigger.h>

#define FLOAT_SIGN(val) (((val) > 0.0) ? +1.0 : -1.0)

using namespace std;

class ArcoPathTracker
{
    typedef actionlib::SimpleActionServer<upo_actions::NavigateAction> NavigationServer;

public:

    
    // Global variables
    ros::Publisher controlPub, speedMarkerPub;

    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;
    //Used to publish the real distance to the goal when arrived(Mostly debugging purposes)
    double x_old, y_old, z_old;

    //Upo actions stuff
    //Navigation server
    std::unique_ptr<NavigationServer> navigate_server_ptr;

    upo_actions::NavigateActionFeedback navFb;
    upo_actions::NavigateResult navResult;
    upo_actions::NavigateGoalConstPtr navGoal;

    //Service clients
    visualization_msgs::Marker speedMarker,rotMarker;
    //Used to set the marker frame
    std::string robot_frame, global_frame_id;
    ros::Time time_count;
    ros::Publisher twistPub;
    ros::ServiceClient check_rot_srv;

    trajectory_msgs::MultiDOFJointTrajectoryPoint  traj_marsupial_;

    geometry_msgs::Twist vel;

    double angle2NextPoint, dist2GlobalGoal, angle2GlobalGoal, dist2NextPoint, angleBack;
    double  linMaxSpeed, linMaxSpeedBack, distMargin, angMaxSpeed, a, b, bBack;
    double Vx, Vy, Wz;
    double angle1, angle2, angle3;

    //Flags
    bool backwards, trajReceived, timeout, navigationPaused,aproximated;

    //Input config params
    bool phase1, phase2;

    geometry_msgs::PoseStamped globalGoalBlFrame, globalGoal, nextPoseBlFrame;

    //For the watchdog
    ros::Time currentT;

    ArcoPathTracker();

    inline float d2rad(float angle);
    inline float rad2d(float angle);
    bool validateRotInPlace();
    void moveNonHolon();
    void navigate();
    void publishCmdVel();

    /**
     * Exponential speed calculator
     * @max: speed at inf
     * @exp_const: the decay constant
     * @var: the variable to be function of
     * v=max*(1-exp(-exp_cons*var))
    **/
    inline float getVel(float max, float exp_const, float var);
    inline float getVel(double max, double exp_const, double var);

    /**
     *  Aux function to get an angle in radians and viceversa
     *  @angle to convert to radians
    **/

    inline double euclideanDistance(double x0, double y0, double x, double y);
    inline double euclideanDistance(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
    inline double euclideanDistance(geometry_msgs::PoseStamped next);

    void publishZeroVel();

    void computeGeometry();

    bool rotationInPlace(geometry_msgs::Quaternion finalOrientation, double threshold_);

    bool rotationInPlace(tf2Scalar dYaw, double threshold_);

    float getYawFromQuat(geometry_msgs::Quaternion quat);

    void fillFeedback(double vx, double vy, double wz, bool sec_stop, std::string text_="ok");

    //Okey, this callback is reached when publishing over Navigate3D/cancel topic a empty message
    void navigatePreemptCallback();

    void navigateGoalCallback();

    void setGoalReachedFlag(bool status_);

    geometry_msgs::PoseStamped transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to);

    geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped originalPose, std::string from, std::string to);

};
#endif 
