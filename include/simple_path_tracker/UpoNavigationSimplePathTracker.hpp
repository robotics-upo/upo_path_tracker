
#ifndef SIMPLE_PATH_TRACKER_H_
#define SIMPLE_PATH_TRACKER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <visualization_msgs/Marker.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <theta_star_2d/checkObstacles.h>

#include <actionlib/server/simple_action_server.h>
#include <upo_actions/NavigateAction.h>
#include <upo_actions/RotationInPlaceAction.h>
#include <upo_actions/ExecuteMissionActionFeedback.h>

#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <upo_path_tracker/SimplePathTrackerConfig.h>

#include "utils/geometry.hpp"
#include "utils/transforms.hpp"


namespace Upo
{
  namespace Navigation
  {
    class SimplePathTracker
    {
        typedef actionlib::SimpleActionServer<upo_actions::NavigateAction> NavigateServer;
        typedef actionlib::SimpleActionServer<upo_actions::RotationInPlaceAction> RotationInPlaceServer;

        /**
         * @brief Generic state machine of the path tracker, each state can have some substates
         * 
         */
        enum NavigationStatus{

          IDLE = 0,
          NAVIGATING_FORMWARD = 1,
          NAVIGATING_BACKWARDS = 2,
          APROXIMATION_MAN_1 = 3,
          APROXIMATION_MAN_2 = 4,
          PATH_TIMEOUT = 5,
          NAVIGATION_PAUSED = 6,
          RECOVERY_ROTATION  = 7

        };

      public: 
        /**
         * @brief Construct a new Simple Path Tracker object
         * 
         */
        SimplePathTracker();
        /**
         * @brief Destroy the Simple Path Tracker object
         * 
         */
        virtual ~SimplePathTracker() = default;

      private:
      
        /**
         * @brief 
         * 
         * @param event 
         */
        void navigate(const ros::TimerEvent &event);
        
        /**
         * @brief 
         * 
         */
        void computeGeometry();
        /**
         * @brief 
         * 
         * @param vx 
         * @param vy 
         * @param wz 
         * @param sec_stop 
         * @param text_ 
         */
        void fillFeedback(const double vx, const double vy, const double wz, const std::string &text);
        /**
         * @brief 
         * 
         */
        void calculateCmdVel();
         /**
         * @brief Set the Goal Reached Flag object
         * 
         * @param status_ 
         */
        void setFinalNavigationStatus(bool arrived_succesfully);
        /**
         * @brief 
         * 
         * @param finalOrientation 
         * @param threshold_ 
         * @return true 
         * @return false 
         */
        bool rotationInPlace(geometry_msgs::Quaternion finalOrientation, double threshold_, bool final);
        /**
         * @brief 
         * 
         * @param dYaw 
         * @param threshold_ 
         * @return true 
         * @return false 
         */
        bool rotationInPlace(tf2Scalar dYaw, double threshold_, bool final);
        /**
         * @brief 
         * 
         * @param thresh 
         * @return true 
         * @return false 
         */
        bool validateRotation(int thresh = 0);
        /**
         * @brief 
         * 
         */
        void publishZeroSpeeds();
        /**
         * @brief 
         * 
         */
        void publishCmdVel();
        /**
         * @brief 
         * 
         */
        void publishMarkers();
        /**
         * @brief 
         * 
         */
        void configureMarkers();
        /**
         * @brief 
         * 
         * @param msg 
         */
        void localPathCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg);
        /**
         * @brief 
         * 
         * @param config 
         * @param level 
         */
        void dynamicReconfigureCallback(upo_path_tracker::SimplePathTrackerConfig &config, uint32_t level);
       

        /**
         * Exponential speed calculator
         * @max: speed at inf
         * @exp_const: the decay constant
         * @var: the variable to be function of
         * v=max*(1-exp(-exp_cons*var))
         **/
        inline float getVel(float max, float exp_const, float var)
        {
          return max * ( 1 - exp( -1.0 * exp_const * fabs(var) ) ) * var / fabs( var );
        }
        inline float getVel(double max, double exp_const, double var)
        {
          return max * ( 1 - exp( -1.0 * exp_const * fabs( var ) ) ) * var / fabs( var );
        }

        //Flags
        bool do_navigate_ = true;

        NavigationStatus status_, status_before_timeout_;

        //Bool parameters
        bool debug_;
        bool holonomic_;
        //Double parameters
        double ang_max_speed_;
        double lin_max_speed_;
        double lin_max_speed_back_;
        double angle_margin_;
        double dist_margin_;
        double a_; 
        double b_; 
        double b_back_;
        double orientdist_;
        double angle1_, angle2_, angle3_;
        double dist_aprox1_;
        double timeout_time_;

        //Int parameters
        int rot_thresh_;
        //String parameters(frames names)
        std::string robot_base_frame_id_;
        std::string world_frame_id_;
        std::string odom_frame_id_;

        // Variables
        // Speed
        double vx_ = 0, vy_ = 0, wz_ = 0;


        double angle_to_next_point_;
        double dist_to_global_goal_;
        double angle_to_global_goal_;
        double dist_to_next_point_;
        double angle_back_;
        geometry_msgs::Twist speed_command_;

        // Trajectory and related
        trajectory_msgs::MultiDOFJointTrajectoryPoint next_point_;
        ros::Time last_trj_stamp_;
        ros::Time time_count_;
        geometry_msgs::PoseStamped global_goal_robot_frame_;
        geometry_msgs::PoseStamped global_goal_;
        geometry_msgs::PoseStamped next_pose_robot_frame_;

        // Node components
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_ { "~" };

        tf2_ros::Buffer tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_list_;

        ros::Publisher twist_pub_;
        ros::Publisher markers_pub_;

        ros::Subscriber local_path_sub_;

        ros::ServiceClient check_rot_srv_;
        ros::ServiceClient costmap_clean_srv;

        ros::Timer navigate_timer_;
        // Markers for RViz
        std::vector<visualization_msgs::Marker> markers_;
        // Security margin pointer

        // Action lib server poitners
        std::unique_ptr<NavigateServer> navigate_server_;
        std::unique_ptr<RotationInPlaceServer> rot_server_;

        // Navigate action objects
        upo_actions::NavigateResult navigate_result_;
        upo_actions::NavigateActionFeedback navigate_fb_;
        upo_actions::NavigateGoalConstPtr navigate_goal_;
        // Rotation action objects
        upo_actions::RotationInPlaceResult rot_result_;
        upo_actions::RotationInPlaceGoalConstPtr rot_inplace_;
        upo_actions::RotationInPlaceActionFeedback rotation_fb_;

        // Dyn reconfg
        std::unique_ptr<dynamic_reconfigure::Server<upo_path_tracker::SimplePathTrackerConfig>> server_;
        std::unique_ptr<dynamic_reconfigure::Server<upo_path_tracker::SimplePathTrackerConfig>::CallbackType> f_;

    };
  }  // namespace Navigation
}  // namespace Upo

#endif /* UPO_NAVIGATION_SIMPLE_PATH_TRACKER_H_ */