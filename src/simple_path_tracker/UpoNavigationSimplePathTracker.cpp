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

            nh_.param("debug", debug_, true);
            nh_.param("holonomic", holonomic_, true);

            nh_.param("angular_max_speed", ang_max_speed_, 0.4);
            nh_.param("linear_max_speed", lin_max_speed_, 0.2);
            nh_.param("linear_max_speed_back", lin_max_speed_back_, 0.2);
            nh_.param("angle_margin", angle_margin_, 10.0);
            nh_.param("dist_margin", dist_margin_, 0.35);
            nh_.param("a", a_, 0.5);
            nh_.param("b", b_, 0.5);
            nh_.param("b_back", b_back_, 0.5);
            nh_.param("start_orientate_dist", orientdist_, 0.5);
            nh_.param("angle1", angle1_, 20.0);
            nh_.param("angle2", angle2_, 65.0);
            nh_.param("angle3", angle3_, 15.0);
            nh_.param("dist_aprox1_", dist_aprox1_, 0.05);

            nh_.param("rot_thresh", rot_thresh_, 350);

            nh_.param("robot_base_frame", robot_base_frame_id_, (std::string) "base_link");
            nh_.param("world_frame_id_", world_frame_id_, (std::string) "map");
            nh_.param("odom_frame", odom_frame_id_, (std::string) "odom");


            costmap_clean_srv = nh_.serviceClient<std_srvs::Trigger>("/custom_costmap_node/reset_costmap");

            // Publishers, only twist and markers_
            twist_pub_ = pnh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            markers_pub_ = pnh_.advertise<visualization_msgs::Marker>("speed_markers", 2);

            local_path_sub_ = nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/local_planner_node/local_path", 2,
                                                                                   &SimplePathTracker::localPathCallback, this);
           
            // Navigate action server configuration
            navigate_server_.reset(new NavigateServer(pnh_, "/SimplePathTrackerActionServer", false));
            navigate_server_->start();

            // Rotation action server configuration
            rot_server_.reset(new RotationInPlaceServer(pnh_, "/Recovery_Rotation", false));
            rot_server_->start();

            // Service to check if it's possible a rotation in place consulting the costmap
            check_rot_srv_ = nh_.serviceClient<theta_star_2d::checkObstacles>("/custom_costmap_node/check_env");

            last_trj_stamp_ = ros::Time(1, 1);

            status_ = NavigationStatus::IDLE;
            // Configure speed direction marker
            configureMarkers();
        }     
        //TODO Clean this function
        void SimplePathTracker::calculateCmdVel()
        {
            angle_back_ = angle_to_next_point_ < 0 ?  angle_to_next_point_ + M_PI: 
                                                      angle_to_next_point_ - M_PI;

          if (dist_to_global_goal_ > dist_margin_ && phase1_)
          {

            ROS_INFO_THROTTLE(0.5, "DIST: %.2f", dist_to_global_goal_);
            ROS_INFO_THROTTLE(0.5, "Angle Back: %.2f", angle_back_);
            ROS_INFO_THROTTLE(0.5, "Backwards: %d", backwards_);
            ROS_INFO_THROTTLE(0.5, "angle to next point: %.2f", angle_to_next_point_);

            if (fabs(angle_to_next_point_) > deg2Rad(angle1_))  // Rot in place
            {
              if (!backwards_ && (fabs(angle_to_next_point_) < deg2Rad(angle2_) || validateRotInPlace()))
              {
                ROS_INFO("\t 1");
                rotationInPlace(angle_to_next_point_, 0, true);
                vx_ = 0;
              }
              else if (!backwards_)
              {
                ROS_INFO("Enabling backwards");
                backwards_ = true;
                time_count_ = ros::Time::now();
              }
              else if (ros::Time::now() - time_count_ > ros::Duration(100) && validateRotInPlace())  // Reset backwards
              {
                ROS_INFO("\t 2");
                backwards_ = false;
              }
              else if (fabs(angle_back_) > fabs(deg2Rad(angle3_)))  // Too much angular distance, do only rotation in place
              {
                ROS_INFO("\t 3");
                rotationInPlace(angle_back_, 0, true);
                vx_ = 0;
              }
              else
              {
                ROS_INFO("\t 4");
                vx_ = -getVel(lin_max_speed_back_, b_back_, dist_to_global_goal_);
                rotationInPlace(angle_back_, 0, false);
              }
            }
            else
            {
              // ROS_INFO("\t 5");
              vx_ = getVel(lin_max_speed_, b_, dist_to_global_goal_);
              vy_ = 0;
              rotationInPlace(angle_to_next_point_, 0, false);
            }
          }
          else if (!aproximated_)
          {
            if (phase1_)
            {
              phase1_ = false;
              std_srvs::Trigger trg;
              costmap_clean_srv.call(trg);
            }
            if (phase2_)  
            {
              wz_ = 0;
              ROS_INFO("Fase 1");

              if (global_goal_robot_frame_.pose.position.x > 0 && global_goal_robot_frame_.pose.position.x < dist_aprox1_)
              {
                vx_ = 0.01;
              }
              else if (global_goal_robot_frame_.pose.position.x < 0 && global_goal_robot_frame_.pose.position.x > -dist_aprox1_)
              {
                vx_ = -0.01;
              }
              else
              {
                ROS_INFO("OUT FASE 1");
                phase2_ = false;
                vx_ = 0;
              }
              // Vx = getVel(global_goal_robot_frame_.pose.position.x < 0 ? -linMaxSpeed : linMaxSpeed, b, dist2GlobalGoal);
            }
            else
            {
              ROS_INFO("2 FASE");

              static geometry_msgs::PoseStamped robotPose;
              static tf2::Quaternion robotQ;
              static tf2::Matrix3x3 m;
              static double robotYaw, rpitch, rroll;

              robotPose.header.frame_id = robot_base_frame_id_;
              robotPose.header.stamp = ros::Time(0);
              robotPose.pose.orientation.w = 1;
              robotPose.pose.orientation.z = 0;
              robotPose = transformPose(robotPose, robot_base_frame_id_, world_frame_id_, tf_buffer_);

              robotQ.setW(robotPose.pose.orientation.w);
              robotQ.setZ(robotPose.pose.orientation.z);
              robotQ.normalize();

              m.setRotation(robotQ);
              m.getEulerYPR(robotYaw, rpitch, rroll);

              ROS_INFO("angle of GlobalGoal: %.2f, robotYaw: %.2f", angle_to_global_goal_, rad2Deg(robotYaw));
              static bool aprox_rot;
              static double rotval;
              if (angle_to_global_goal_ * rad2Deg(robotYaw) < 0)
              {
                if (angle_to_global_goal_ < 0)
                {
                  rotval = (180 + angle_to_global_goal_) + (180 - rad2Deg(robotYaw));
                }
                else
                {
                  rotval = -((180 - angle_to_global_goal_) + (180 + rad2Deg(robotYaw)));
                }
              }
              else
              {
                rotval = angle_to_global_goal_ - rad2Deg(robotYaw);
              }
              if (fabs(rotval) > 180)
              {
                if (rotval < 0)
                {
                  rotval += 360;
                }
                else
                {
                  rotval -= 360;
                }
              }
              ROS_INFO("Rotation value: %.2f", rotval);
              if (validateRotInPlace(rot_thresh_))
              {
                aprox_rot = rotationInPlace(deg2Rad(rotval), 5, true);
                if (!aprox_rot)
                {
                  aproximated_ = true;
                  setGoalReachedFlag(1);
                  ROS_INFO("Aproximated");
                }
              }
              else
              {
                aproximated_ = true;
                setGoalReachedFlag(1);
                ROS_WARN("Arrived but not aproximated");
              }
            }
          }
        }  // namespace Navigators
        bool SimplePathTracker::rotationInPlace(geometry_msgs::Quaternion finalOrientation, double threshold_,
                                                bool final = false)
        {
          geometry_msgs::PoseStamped robotPose;
          tf2::Quaternion finalQ, robotQ;

          robotPose.header.frame_id = robot_base_frame_id_;
          robotPose.header.stamp = ros::Time::now();
          robotPose.pose.orientation.w = 1;

          robotPose = transformPose(robotPose, robot_base_frame_id_, world_frame_id_, tf_buffer_);

          robotQ.setW(robotPose.pose.orientation.w);
          robotQ.setZ(robotPose.pose.orientation.z);

          finalQ.setW(finalOrientation.w);
          finalQ.setZ(finalOrientation.z);

          // This give us the angular difference to the final orientation
          tf2Scalar shortest = tf2::angleShortestPath(robotQ, finalQ);
          double sh = static_cast<double>(shortest);
          std::cout << "Shortest: " << sh << std::endl;

          return rotationInPlace(shortest, threshold_, final);
        }
        bool SimplePathTracker::rotationInPlace(tf2Scalar dYaw, double threshold_, bool final = false)
        {
          bool ret = true;

          tf2::Quaternion q_rot, q_f, robotQ;
          geometry_msgs::PoseStamped robotPose;

          robotPose.header.frame_id = robot_base_frame_id_;
          robotPose.header.stamp = ros::Time(0);
          robotPose.pose.orientation.w = 1;
          robotPose = transformPose(robotPose, robot_base_frame_id_, world_frame_id_, tf_buffer_);

          robotQ.setW(robotPose.pose.orientation.w);
          robotQ.setZ(robotPose.pose.orientation.z);

          q_rot.setRPY(0, 0, dYaw);

          q_f = robotQ * q_rot;
          q_f.normalize();

          double rotation = static_cast<double>(dYaw);  // radians
          // std::cout << "Rotation rotation: " << rad2Deg(rotation) << std::endl;
          if (fabs(rotation) > deg2Rad(threshold_))
          {
            std::clamp(rotation,-M_PI_2, M_PI_2);
            wz_ = getVel(final ? ang_max_speed_ + 0.05 : ang_max_speed_, final ? a_ / 2 : a_, rad2Deg(rotation));  // TODO poner el 0.2 como parametro
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
        //TODO clean
        bool SimplePathTracker::checkPathTimeout()
        {
          bool ret = false;
          if (!navigate_server_->isActive() || !traj_received_ || ros::Time::now() - last_trj_stamp_ > ros::Duration(2))
          {
            publishZeroSpeeds();
            ret = true;
          }

          return ret;
        }  // namespace Navigators
        //TODO clean
        void SimplePathTracker::navigate()
        {
          if (rot_server_->isNewGoalAvailable())
          {
            rot_inplace_ = rot_server_->acceptNewGoal();
          }
          if(navigate_server_->isPreemptRequested()){
            navigate_server_->setPreempted();
            //TODO sustituir los flags por la maquina de estados
            status_ = NavigationStatus::IDLE;

            backwards_ = false;
            recovery_rotation_ = false;
            aproximated_ = false;
            phase2_ = true;
            phase1_ = true;
            // Flags for internal states
            traj_received_ = false;
            vx_ = vy_ = wz_ = 0;
          }
          if (navigate_server_->isNewGoalAvailable())
          {
            ROS_INFO("Accepting new goal ");
            navigate_goal_ = navigate_server_->acceptNewGoal();
            global_goal_.pose = navigate_goal_->global_goal;
            std::cout << "global_goal_: " << global_goal_.pose.position.x << " " << global_goal_.pose.position.y << " "
                 << global_goal_.pose.orientation.z << " " << global_goal_.pose.orientation.w << std::endl;

            time_count_ = ros::Time::now();

            setGoalReachedFlag(0);
          }
          if (!checkPathTimeout() && (navigate_server_->isActive() || !aproximated_) )
          {
            computeGeometry();

            calculateCmdVel();
         
            publishCmdVel();
          }
        }
        void SimplePathTracker::publishCmdVel()
        {

            if (navigation_paused_)
              navigation_paused_ = false;
            
            fillFeedback(vx_, vy_, wz_, "ok");

            speed_command_.angular.z = wz_;
            speed_command_.linear.x = vx_;
            speed_command_.linear.y = vy_;

            twist_pub_.publish(speed_command_);
            publishMarkers();
          
        }
        bool SimplePathTracker::validateRotInPlace(int thresh)
        {
          bool ret = false;

          theta_star_2d::checkObstacles trg;
          trg.request.thresh.data = thresh;
          check_rot_srv_.call(trg);
          
          if (trg.response.success)
            ret = true;

          return ret;
        }
        void SimplePathTracker::setGoalReachedFlag(bool status_)
        {
          if (status_ && !navigate_result_.arrived)
          {
            navigate_result_.arrived = true;
            traj_received_ = false;

            navigate_result_.finalAngle.data = angle_to_global_goal_;
            navigate_result_.finalDist.data = dist_to_global_goal_;
            navigate_server_->setSucceeded(navigate_result_, "Goal reached succesfully");
           
            publishZeroSpeeds();
          }
          else if (!status_ && navigate_result_.arrived)
          {
            navigate_result_.arrived = false;
            backwards_ = false;
            aproximated_ = false;
            phase2_ = true;
            phase1_ = true;
          }
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
            
            if (navigate_server_->isActive())
            {
              next_point_ = msg->points[msg->points.size() > 1 ? 1 : 0];
              traj_received_ = true;
              last_trj_stamp_ = ros::Time::now();
            }
        }
        void SimplePathTracker::dynamicReconfigureCallback(upo_path_tracker::SimplePathTrackerConfig &config, uint32_t level)
        {
          angle1_ = config.angle1;
          angle2_ = config.angle2;
          angle3_ = config.angle3;

          holonomic_ = config.holonomic;
          do_navigate_ = config.do_navigate;
          
          ang_max_speed_ = config.angular_max_speed;
          lin_max_speed_ = config.linear_max_speed;
          lin_max_speed_back_ = config.linear_max_speed_back;
          
          dist_margin_ = config.dist_margin;

          a_ = config.a;
          b_ = config.b;
          b_back_ = config.b_back;
        }

    }
}