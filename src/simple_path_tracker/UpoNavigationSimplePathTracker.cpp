#include <simple_path_tracker/UpoNavigationSimplePathTracker.hpp>

using namespace Upo::Utils::Geometry;
using namespace Upo::Utils::Transforms;
using namespace Upo::Navigation;

namespace Upo{
    namespace Navigation{

        SimplePathTracker::SimplePathTracker(){

            tf2_list_.reset(new tf2_ros::TransformListener(tf_buffer_));

            server_.reset(new dynamic_reconfigure::Server<upo_path_tracker::SimplePathTrackerConfig>);
            f_.reset(new dynamic_reconfigure::Server<upo_path_tracker::SimplePathTrackerConfig>::CallbackType);
            *f_ = boost::bind(&SimplePathTracker::dynamicReconfigureCallback, this, _1, _2);
            server_->setCallback(*f_);

            ROS_INFO("Tracker: Loading params...");

            nh_.param("angular_max_speed", ang_max_speed_, 0.8);
            nh_.param("linear_max_speed", lin_max_speed_, 0.25);

            nh_.param("linear_max_speed_back", lin_max_speed_back_, 0.25);
            nh_.param("angle_margin", angle_margin_, 10.0);
            nh_.param("start_aproximation_distance", aprox_distance_, 0.3);
            nh_.param("a", a_, 1.4);
            nh_.param("b", b_, 0.5);
            nh_.param("b_back", b_back_, 0.5);
            nh_.param("dist_aprox1_", dist_aprox1_, 0.05);
            nh_.param("local_paths_timeout", timeout_time_, 2.0);
            nh_.param("angle1", angle1_, 35.0);
            nh_.param("angle2", angle2_, 65.0);
            nh_.param("angle3", angle3_, 15.0);

            nh_.param("rot_thresh", rot_thresh_, 350);
            nh_.param("force_rotation", force_rotation_, false);
            nh_.param("force_final_rotation", force_final_rotation_, true);

            nh_.param("robot_base_frame", robot_base_frame_id_, (std::string) "base_link");
            nh_.param("world_frame_id", world_frame_id_, (std::string) "map");
            nh_.param("odom_frame", odom_frame_id_, (std::string) "odom");
            double rate;
            nh_.param("rate", rate, 40.0);
            navigate_timer_ = nh_.createTimer(ros::Duration(1/rate), &SimplePathTracker::processActionsStatus, this);

            double back_dur;
            nh_.param("backwards_duration", back_dur, 30.0);
            backwards_duration_ = ros::Duration(back_dur);
            ROS_INFO("Tracker: Configuring topics...");
            // Publishers, only twist and markers_
            twist_pub_ = pnh_.advertise<geometry_msgs::Twist>("/nav_vel", 1);
            markers_pub_ = pnh_.advertise<visualization_msgs::Marker>("speed_markers", 2);

            local_path_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/local_planner_node/local_path", 2,
                                                                                   &SimplePathTracker::localPathCallback, this);
           
            // Navigate action server configuration
            navigate_server_.reset(new NavigateServer(pnh_, "/Navigation", false));
            navigate_server_->start();

            // Rotation action server configuration
            rot_server_.reset(new RotationInPlaceServer(pnh_, "/Recovery_Rotation", false));
            rot_server_->start();

            // Service to check if it's possible a rotation in place consulting the costmap
            check_rot_srv_ = nh_.serviceClient<nix_common::CheckObstacles>("/custom_costmap_node/check_env");
            costmap_clean_srv = nh_.serviceClient<std_srvs::Trigger>("/custom_costmap_node/reset_costmap");

            // Configure speed direction marker
            configureMarkers();
        }     
        void SimplePathTracker::calculateCmdVel()
        {
          if(!new_path_)
            return;

          computeGeometry();
          
          switch (status_)
          {
            case NAVIGATING_FORWARD:
            {
              ROS_INFO("Navigating forward");
              if(dist_to_global_goal_ > aprox_distance_){
                  ROS_INFO("Angle to next: %.2f , angle1: %.2f, angle2: %.2f",rad2Deg(std::fabs(angle_to_next_point_)) , angle1_,angle2_);

                if (std::fabs(angle_to_next_point_) > deg2Rad(angle1_))  // Rot in place
                {
                  if ( (fabs(angle_to_next_point_) < deg2Rad(angle2_) || force_rotation_ || validateRotation(rot_thresh_)) )
                  {
                    rotationInPlace(angle_to_next_point_, deg2Rad(5), true);
                    vx_ = 0;
                  }
                  else
                  {
                    status_ = NavigationStatus::NAVIGATING_BACKWARDS;
                    backwards_time_counter_ = ros::Time::now();
                    ROS_INFO("Switching to BACKWARDS: %.2f > %.2f", fabs(angle_to_next_point_), angle2_);
                  }

                }else{

                  vx_ = getVel(lin_max_speed_, b_, dist_to_global_goal_);
                  vy_ = 0;
                  rotationInPlace(angle_to_next_point_, 0, false);
                }

              }else{
                previous_status_ = status_;
                status_ = NavigationStatus::APROXIMATION_MAN_1;
                std_srvs::Trigger trg;
                costmap_clean_srv.call(trg);
              }
              publishCmdVel();

              break;
            }
            case NAVIGATING_BACKWARDS:
            {
                ROS_INFO("Navigating backwards");

                angle_back_ = angle_to_next_point_ < 0 ?  angle_to_next_point_ + M_PI: 
                                                            angle_to_next_point_ - M_PI;
                if(dist_to_global_goal_ > aprox_distance_){
                  
                  if (ros::Time::now() - backwards_time_counter_ > backwards_duration_ && validateRotation(rot_thresh_)) 
                  {
                    status_ = NavigationStatus::NAVIGATING_FORWARD;
                  }
                  else if ( std::fabs(angle_back_) > std::fabs( deg2Rad(angle3_) ) )  // Too much angular distance, do only rotation in place
                  {
                    rotationInPlace(angle_back_, 0, true);
                    vx_ = 0;
                  }
                  else
                  {
                    vx_ = -getVel(lin_max_speed_back_, b_back_, dist_to_global_goal_);
                    rotationInPlace(angle_back_, 0, false);
                  }
                }else{
                  previous_status_ = status_;
                  status_ = NavigationStatus::APROXIMATION_MAN_1;
                  std_srvs::Trigger trg;
                  costmap_clean_srv.call(trg);
                }
              publishCmdVel();

              break;
            }
            case APROXIMATION_MAN_1:
            {
              // ROS_INFO("Approx Man 1");
              wz_ = 0;
              vx_ = getVel(lin_max_speed_, b_, dist_to_global_goal_);
              
              double dist = global_goal_robot_frame_.pose.position.x;
              // ROS_INFO("Dist: %f", dist);
              std::clamp(vx_, -1 * dist, dist);
              vx_ /= 1.5;

              if(previous_status_ == NavigationStatus::NAVIGATING_BACKWARDS)
                vx_ *= -1;

              if(std::fabs(dist) < dist_aprox1_){
	        vx_ = 0;	
                status_ = NavigationStatus::APROXIMATION_MAN_2;
	      }
              publishCmdVel();

              break;
            }
            case APROXIMATION_MAN_2:
            {
              // ROS_INFO("Approx Man 2");
              double robotYaw, rpitch, rroll;
              getCurrentOrientation(tf_buffer_, robot_base_frame_id_, world_frame_id_).getEulerYPR(robotYaw, rpitch, rroll);
              
              // std::cout<<"Robot yaw: "<<robotYaw<<std::endl;
              // std::cout<<"Angle 2gg: "<<angle_to_global_goal_<<std::endl;
              
              double rotval =  deg2Rad(angle_to_global_goal_) - robotYaw;

              if(angle_to_global_goal_ > deg2Rad(M_PI_2) && robotYaw < 0 ) 
                rotval =  -1* (2* M_PI - deg2Rad(angle_to_global_goal_) + robotYaw);

              if(std::isnan(robotYaw) || std::isnan(rotval))
                return;

              // std::cout<<"rotval: "<<rotval<<std::endl;
              // std::cout<<"angle margin: "<<angle_margin_<<std::endl;

              removeMultipleRotations(rotval);

              if (force_final_rotation_ || validateRotation(rot_thresh_))
              {
                ROS_INFO("Rotation Validated!");
                if( !rotationInPlace(rotval, deg2Rad(angle_margin_), true) ) 
                
                  setFinalNavigationStatus(true);
              
              }
              else
              {
                ROS_INFO("Not possible to make a rotation !");
                setFinalNavigationStatus(true);
              }
              publishCmdVel();
              break;
            }
            default:
            {
              publishZeroSpeeds();
              break;
            }
          }

        } 
        bool SimplePathTracker::rotationInPlace(const geometry_msgs::Quaternion &final_orientation,const double &threshold,
                                                bool final = false)
        {
          ROS_INFO("Performing rotation in place (1)");

          tf2::Quaternion final_orientation_q, robot_orientation;
          final_orientation_q.setW(final_orientation.w);
          final_orientation_q.setZ(final_orientation.z);
          
          getCurrentOrientation(tf_buffer_).getRotation(robot_orientation);

          // This give us the angular difference to the final orientation
          tf2Scalar shortest = tf2::angleShortestPath(robot_orientation, final_orientation_q);

          return rotationInPlace(shortest, threshold, final);
        }
        bool SimplePathTracker::rotationInPlace(const double &diff_yaw,const double &threshold, bool final = false)
        {
          ROS_INFO("Performing rotation in place (2)");
          bool ret = true;
          // double diff_yaw_rad = deg2Rad(diff_yaw);

          ROS_INFO("Yaw diff: %.2f , threshold: %.f", diff_yaw, threshold);
          if (std::fabs(diff_yaw) > threshold)
          {
            // double diff_yaw_rad = deg2Rad(diff_yaw);
            std::clamp(diff_yaw,-M_PI, M_PI);
            wz_ = getVel(final ? ang_max_speed_ + 0.05 : ang_max_speed_, final ? a_ / 2 : a_, diff_yaw); 
            // wz_ = getVel(final ? ang_max_speed_ + 0.05 : ang_max_speed_, a_, diff_yaw_rad); 
            
            ROS_INFO("Wz: %.2f / %.2f", wz_, ang_max_speed_);
          }
          else
          {
            wz_ = 0;
            ret = false;
          }

          return ret;
        }
        void SimplePathTracker::computeGeometry()
        {
          next_pose_robot_frame_ = transformPose(next_point_, world_frame_id_, robot_base_frame_id_, tf_buffer_);
          global_goal_robot_frame_ = transformPose(global_goal_, world_frame_id_, robot_base_frame_id_, tf_buffer_);
          angle_to_global_goal_ = getYawFromQuat(global_goal_.pose.orientation);
          dist_to_global_goal_ = euclideanDistance(global_goal_robot_frame_);
          angle_to_next_point_ = atan2(next_pose_robot_frame_.pose.position.y, next_pose_robot_frame_.pose.position.x);
          dist_to_next_point_ = euclideanDistance(next_pose_robot_frame_);
        }
        void SimplePathTracker::processActionsStatus(const ros::TimerEvent &event)
        {
          
          if( rot_server_->isNewGoalAvailable() ) 
          {
            rot_inplace_ = rot_server_->acceptNewGoal();
            status_ = NavigationStatus::RECOVERY_ROTATION;
            ROS_INFO("Tracker: Status set to Recovery rotation");
          
          }
          if( rot_server_->isPreemptRequested() ){
            rot_server_->setPreempted();
            status_ = NavigationStatus::IDLE;
            ROS_INFO("Tracker: Status set to Idle");

          } 

          if (navigate_server_->isNewGoalAvailable())
          {
            navigate_goal_ = navigate_server_->acceptNewGoal();
            global_goal_.pose = navigate_goal_->global_goal;
            last_trj_stamp_ = ros::Time::now();
            status_ = NavigationStatus::NAVIGATING_FORWARD;

            ROS_INFO("Tracker: New goal accepted ");
            
          }
          
          if(navigate_server_->isPreemptRequested()){
            ROS_INFO("Navigation preempted by external request");
            setFinalNavigationStatus(false);
            // navigate_server_->setPreempted();
          }


          if ( navigate_server_->isActive() )          
            calculateCmdVel();
          
        }
        void SimplePathTracker::publishCmdVel()
        {
            
            fillFeedback(vx_, vy_, wz_, "ok");

            speed_command_.angular.z = wz_;
            speed_command_.linear.x = vx_;
            speed_command_.linear.y = vy_;

            twist_pub_.publish(speed_command_);
            publishMarkers();
          
        }
        bool SimplePathTracker::validateRotation(int thresh)
        {
          bool ret = false;

          nix_common::CheckObstacles trg;
          trg.request.thresh.data = thresh;
          check_rot_srv_.call(trg);
          
          if (trg.response.success)
            ret = true;

          return ret;
        }
        void SimplePathTracker::setFinalNavigationStatus(bool arrived_succesfully)
        {
          ROS_INFO("Tracker: Setting final navigation status: %d", (int)arrived_succesfully);

          if (arrived_succesfully)
          {
            navigate_result_.arrived = true;

            navigate_result_.finalAngle.data = angle_to_global_goal_;
            navigate_result_.finalDist.data = dist_to_global_goal_;

            navigate_server_->setSucceeded(navigate_result_, "Goal reached succesfully");
          }
          else
          {
            navigate_result_.arrived = false;
          }
          new_path_ = false;
          status_ = NavigationStatus::IDLE;
          publishZeroSpeeds();

        }
        void SimplePathTracker::fillFeedback(const double vx, const double vy, const double wz, const std::string &text)
        {
            
            navigate_fb_.header.stamp = ros::Time::now();
            navigate_fb_.header.seq++;
            
            navigate_fb_.feedback.distance_to_goal.data = dist_to_global_goal_;

            navigate_fb_.feedback.speed.x = vx_;
            navigate_fb_.feedback.speed.y = vy_;
            navigate_fb_.feedback.speed.z = wz_;

            navigate_fb_.status.text = text;
            
            navigate_server_->publishFeedback(navigate_fb_.feedback);
        }
        void SimplePathTracker::publishZeroSpeeds()
        {
            ROS_INFO("Publishing Zero Speeds");
            
            vx_ = vy_ = wz_ = 0; 

            speed_command_.angular.z = wz_;
            speed_command_.linear.x = vx_;
            speed_command_.linear.y = vy_;  
            
            publishMarkers();
            twist_pub_.publish(speed_command_);
        }
        void SimplePathTracker::configureMarkers()
        {
            markers_.resize(2);
            // marke Speed markers_
            markers_.at(0).header.frame_id = robot_base_frame_id_;
            markers_.at(0).header.stamp = ros::Time::now();
            markers_.at(0).ns = "simple_path_tracker";
            markers_.at(0).id = 1;
            markers_.at(0).type = visualization_msgs::Marker::ARROW;
            markers_.at(0).action = visualization_msgs::Marker::ADD;
            markers_.at(0).lifetime = ros::Duration(1);
            markers_.at(0).scale.y = 0.1;
            markers_.at(0).scale.z = 0.2;
            markers_.at(0).pose.position.x = 0;
            markers_.at(0).pose.position.y = 0;
            markers_.at(0).pose.position.z = 1;
            markers_.at(0).color.a = 1.0;
            markers_.at(0).color.b = 1.0;
            markers_.at(0).color.g = 1.0;
            markers_.at(0).color.r = 0.0;
            // Angular Speed Arrow
            markers_.at(1).header.frame_id = robot_base_frame_id_;
            markers_.at(1).header.stamp = ros::Time::now();
            markers_.at(1).ns = "simple_path_tracker";
            markers_.at(1).id = 2;
            markers_.at(1).type = visualization_msgs::Marker::ARROW;
            markers_.at(1).action = visualization_msgs::Marker::ADD;
            markers_.at(1).lifetime = ros::Duration(1);
            markers_.at(1).scale.y = 0.1;
            markers_.at(1).scale.z = 0.2;
            markers_.at(1).pose.position.x = 0;
            markers_.at(1).pose.position.y = 0;
            markers_.at(1).pose.position.z = 1;
            markers_.at(1).color.a = 1.0;
            markers_.at(1).color.b = 1.0;
            markers_.at(1).color.g = 0.0;
            markers_.at(1).color.r = 1.0;

            tf2::Quaternion quat;
            quat.setRPY(0, M_PI_2, 0);
            markers_.at(1).pose.orientation.x = quat.getX();
            markers_.at(1).pose.orientation.y = quat.getY();
            markers_.at(1).pose.orientation.z = quat.getZ();
            markers_.at(1).pose.orientation.w = quat.getW();
            ROS_INFO("Tracker: Markers Configured");
        }   
        void SimplePathTracker::publishMarkers()
        {
          static tf2::Quaternion quat;

          quat.setRPY(0, 0, atan2(vy_, vx_));  // Create this quatern

          markers_.at(0).scale.x = 2 * sqrtf(vx_ * vx_ + vy_ * vy_); //The 2 is a scale factor in order to make a bigger and visible marker
          markers_.at(0).pose.orientation.z = quat.getZ();
          markers_.at(0).pose.orientation.w = quat.getW();

          markers_.at(1).scale.x = -2 * wz_ ;  // Minus sign is to follow right hand rule

          markers_pub_.publish(markers_.at(0));
          markers_pub_.publish(markers_.at(1));
        }
        void SimplePathTracker::localPathCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg)
        {
            // ROS_INFO("Tracker: Local Path Callback");

            if (navigate_server_->isActive() && status_ != NavigationStatus::NAVIGATION_PAUSED)
            {
              next_point_ = msg->points[msg->points.size() > 1 ? 1 : 0];
              new_path_ = true;
              //Check timeou
              if(  ros::Time::now() - last_trj_stamp_ > ros::Duration(timeout_time_)  && status_ != NavigationStatus::APROXIMATION_MAN_1 && status_ != NavigationStatus::APROXIMATION_MAN_2 ){ 

                status_before_timeout_ = status_;
                last_trj_stamp_ = ros::Time::now();
                status_ = NavigationStatus::PATH_TIMEOUT;
                ROS_INFO("Tracker: Path Timeout. ");

              }else if (status_ == NavigationStatus::PATH_TIMEOUT ) {
                last_trj_stamp_ = ros::Time::now();
                status_ = status_before_timeout_;
                ROS_INFO("Tracker: Back from timeout");

              }else{
                last_trj_stamp_ = ros::Time::now();
              }

            }
        }
        void SimplePathTracker::dynamicReconfigureCallback(upo_path_tracker::SimplePathTrackerConfig &config, uint32_t level)
        {
          ROS_INFO("Dynamic reconfigure callback");
          
          angle1_ = config.angle1;
          angle2_ = config.angle2;
          angle3_ = config.angle3;

          ang_max_speed_ = config.angular_max_speed;
          lin_max_speed_ = config.linear_max_speed;
          lin_max_speed_back_ = config.linear_max_speed_back;
          
          aprox_distance_ = config.dist_margin;

          a_ = config.a;
          b_ = config.b;
          b_back_ = config.b_back;

          do_navigate_ = config.do_navigate;
          
          if( do_navigate_ && status_ == NavigationStatus::NAVIGATION_PAUSED ){//TODO Not tested
            status_ = status_before_timeout_;
          }
          if( !do_navigate_ ){
            status_before_timeout_ = status_;
            status_ = NavigationStatus::NAVIGATION_PAUSED;
          }
        }
    }
}
