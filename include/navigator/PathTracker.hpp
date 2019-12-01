
#ifndef PATHTRACKER_H_
#define PATHTRACKER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <navigator/securityMargin.hpp>

#include <upo_actions/NavigateAction.h>
#include <upo_actions/RotationInPlaceAction.h>

#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/server.h>
#include <arco_path_tracker/PathTrackerConfig.h>


class PathTracker
{
    typedef actionlib::SimpleActionServer<upo_actions::NavigateAction> NavigateServer;
    typedef actionlib::SimpleActionServer<upo_actions::RotationInPlaceAction> RotationInPlaceServer;

public:
    PathTracker();

    void navigate();

private:
    void dynReconfCb(arco_path_tracker::PathTrackerConfig &config, uint32_t level);
    void computeGeometry();
    bool checkPathTimeout();
    void publishMarkers();
    void fillFeedback(double vx, double vy, double wz, bool sec_stop, std::string text_="ok");
    void moveHolon(double finalYaw);
    void moveNonHolon();

    void setRobotOrientation(float finalYaw, bool goal, bool pub, float speed, float angleMargin_);
    bool rotationInPlace(geometry_msgs::Quaternion finalOrientation, double threshold_);
    bool rotationInPlace(tf2Scalar dYaw, double threshold_);
    bool validateRotInPlace();

    void publishZeroVel();
    void publishCmdVel();
    void configureMarkers();

    //Callbacks
    void localPathCb(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);

    //Action lib Navigation callbacks
    void navGoalCb();
    void navPreemptCb();

    //Action lib Rotation in place Callbacks
    void rotGoalCb();
    void rotPreemptCb();
    geometry_msgs::PoseStamped transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to);
    geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped originalPose, std::string from, std::string to);

    float getYawFromQuat(geometry_msgs::Quaternion quat);
    void setGoalReachedFlag(bool status_);
    inline double euclideanDistance(double x0, double y0, double x, double y)
    {
        return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
    }
    inline double euclideanDistance(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
    {
        return euclideanDistance(pose1.pose.position.x, pose1.pose.position.y, pose2.pose.position.x, pose2.pose.position.y);
    }
    inline double euclideanDistance(geometry_msgs::PoseStamped next)
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
    inline float getVel(double max, double exp_const, double var)
    {
        return max * (1 - exp(-exp_const * fabs(var))) * var / fabs(var);
    }

    //Flags
    bool backwards, recoveryRotation, trajReceived, timeout, navigationPaused,aproximated;
    
    //Input config params
    bool debug, doNavigate, holon, phase1, phase2;
    double angMaxSpeed, linMaxSpeed,linMaxSpeedBack, angleMargin, distMargin, a, b,bBack, orientDist;
    std::string robot_frame, world_frame;

    //Variables
    //tf2::Quaternion robotQ, finalQ, q_rot, q_f;
    //Speed
    double Vx, Vy, Wz;
    double angle2NextPoint, dist2GlobalGoal, angle2GlobalGoal, dist2NextPoint,angleBack;
    geometry_msgs::Twist vel;

    //Trajectory and related
    trajectory_msgs::MultiDOFJointTrajectoryPoint nextPoint;
    ros::Time last_trj_stamp, time_count;
    geometry_msgs::PoseStamped globalGoalBlFrame, globalGoal,nextPoseBlFrame;
    
    //Node components
    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;

    ros::Publisher twistPub, markersPub;
    ros::Subscriber localPathSub;
    ros::ServiceClient check_rot_srv;

    //Markers for RViz
    std::vector<visualization_msgs::Marker> markers;
    //Security margin pointer
    std::unique_ptr<SecurityMargin> margin;

    //Action lib server poitners
    std::unique_ptr<NavigateServer> navigate_server_ptr;
    std::unique_ptr<RotationInPlaceServer> rot_server_ptr;
    //Navigate action objects
    upo_actions::NavigateResult navigate_result;
    upo_actions::NavigateActionFeedback navigate_fb;
    upo_actions::NavigateGoalConstPtr navigate_goal;
    //Rotation action objects
    upo_actions::RotationInPlaceResult rot_result;
    upo_actions::RotationInPlaceGoalConstPtr rot_inplace;
    upo_actions::RotationInPlaceActionFeedback rotation_fb;

    //angular parameters
    double angle1, angle2, angle3;

    //Dyn reconfg
    std::unique_ptr<dynamic_reconfigure::Server<arco_path_tracker::PathTrackerConfig>> server;
    std::unique_ptr<dynamic_reconfigure::Server<arco_path_tracker::PathTrackerConfig>::CallbackType> f;
};

#endif /* DISPLACEMENT_H_ */