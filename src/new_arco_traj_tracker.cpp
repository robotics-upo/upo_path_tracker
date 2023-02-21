#include <navigator/new_arco_traj_tracker.hpp>


ArcoPathTracker::ArcoPathTracker()
{
  
  nh.reset(new ros::NodeHandle("~"));
  tfBuffer.reset(new tf2_ros::Buffer);
  tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));


  nh->param("global_frame_id", global_frame_id, static_cast<std::string>("world"));
  
  ROS_INFO("Using UGV frame %s. Global frame: %s",robot_frame.c_str(), global_frame_id.c_str());

  nh->param("linear_max_speed", linMaxSpeed, (double)0.2);
  nh->param("linear_max_speed_back", linMaxSpeedBack, (double)0.2);
  nh->param("angular_max_speed", angMaxSpeed, (double)1.0);

  nh->param("dist_margin", distMargin, (double)0.35);
  nh->param("a", a, (double)0.5);
  nh->param("b", b, (double)0.5);
  nh->param("b_back", bBack, (double)0.5);
  nh->param("robot_base_frame", robot_frame, (string) "base_link");

  nh->param("angle1", angle1, (double)20);
  nh->param("angle2", angle2, (double)65);
  nh->param("angle3", angle3, (double)15);

  //Publish speed arrow marker
  twistPub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  speedMarkerPub = nh->advertise<visualization_msgs::Marker>("speedMarker", 2);
  controlPub = nh->advertise<sensor_msgs::Joy>("/joy", 0);    


	
  //Start action server to communicate with local planner
  navigate_server_ptr.reset(new NavigationServer(*nh,"/Navigation",false));
  navigate_server_ptr->registerGoalCallback(boost::bind(&ArcoPathTracker::navigateGoalCallback,this));
  navigate_server_ptr->registerPreemptCallback(boost::bind(&ArcoPathTracker::navigatePreemptCallback,this));
  navigate_server_ptr->start();
}

inline float ArcoPathTracker::d2rad(float angle)
{
    return angle / 180 * M_PI;
}

inline float ArcoPathTracker::rad2d(float angle)
{
    return angle / M_PI * 180;
}

bool ArcoPathTracker::validateRotInPlace()
{
    bool ret = false;
    std_srvs::Trigger srv;
    check_rot_srv.call(srv);
    ROS_WARN("Validate rotation requested: %s", srv.response.message.c_str());
    if (srv.response.success)
        return true;

    return ret;
}

void ArcoPathTracker::moveNonHolon()
{
  ROS_INFO_THROTTLE(0.5, "angle2NextPoint: %.2f", angle2NextPoint);
  
  if (angle2NextPoint < 0)
  {
      angleBack = angle2NextPoint;
  }
  else
  {
      angleBack = angle2NextPoint;
  }
  if (dist2GlobalGoal > distMargin && phase1)
  {
    ROS_INFO_THROTTLE(0.5, "DIST: %.2f", dist2GlobalGoal);
    ROS_INFO_THROTTLE(0.5, "Angle Back: %.2f", angleBack);
    ROS_INFO_THROTTLE(0.5, "Backwards: %d", backwards);
    ROS_INFO_THROTTLE(0.5, "angle to next point: %.2f", angle2NextPoint);
    if (fabs(angle2NextPoint) > d2rad(angle1)) //Rot in place
    {
      if (!backwards && (fabs(angle2NextPoint) < d2rad(angle2) || validateRotInPlace()))
      {
          ROS_INFO("\t 1");
          rotationInPlace(angle2NextPoint, 0);
          Vx = 0;
      }
      else if (!backwards)
      {
          ROS_INFO("Enabling backwards");
          backwards = true;
          time_count = ros::Time::now();
      }
      else if (ros::Time::now() - time_count > ros::Duration(15) && validateRotInPlace()) //Reset backwards
      {
          ROS_INFO("\t 2");
          backwards = false;
      }
      else if (fabs(angleBack) > fabs(d2rad(angle3))) //Too much angular distance, do only rotation in place
      {
          ROS_INFO("\t 3");
          rotationInPlace(angleBack, 0);
          Vx = 0;
      }
      else
      {
          ROS_INFO("\t 4");
          Vx = getVel(linMaxSpeedBack / 1.2, bBack / 2, dist2GlobalGoal);
          rotationInPlace(angleBack, 0);
      }
    }
    else
    {
        ROS_INFO("\t 5");
        Vx = getVel(linMaxSpeed, b, dist2GlobalGoal);
        Vy = 0;
        rotationInPlace(angle2NextPoint, 0);
    }
  }
  else if (!aproximated)
  {
    if (phase1)
      phase1 = false;
    if (phase2) 
    {
      ROS_INFO("Fase 1");
      if (globalGoalBlFrame.pose.position.x > 0 && globalGoalBlFrame.pose.position.x < 0.1)
      {
          Vx = 0.05;
      }
      else if (globalGoalBlFrame.pose.position.x < 0 && globalGoalBlFrame.pose.position.x > -0.1)
      {
          Vx = -0.05;
      }
      else
      {
          ROS_INFO("OUT FASE 1");
          phase2 = false;
          Vx = 0;
      }
    }
    else
    {
      ROS_INFO("2 FASE");

      static geometry_msgs::PoseStamped robotPose;
      static tf2::Quaternion robotQ;
      static tf2::Matrix3x3 m;
      static double robotYaw, rpitch, rroll;

      robotPose.header.frame_id = robot_frame;
      robotPose.header.stamp = ros::Time(0);
      robotPose.pose.orientation.w = 1;
      robotPose.pose.orientation.z = 0;
      robotPose = transformPose(robotPose, robot_frame, global_frame_id);

      robotQ.setW(robotPose.pose.orientation.w);
      robotQ.setZ(robotPose.pose.orientation.z);
      robotQ.normalize();

      m.setRotation(robotQ);
      m.getEulerYPR(robotYaw, rpitch, rroll);

      ROS_INFO("angle of GlobalGoal: %.2f, robotYaw: %.2f", angle2GlobalGoal, rad2d(robotYaw));
      static bool aprox_rot;
      static double rotval;
      if (angle2GlobalGoal * rad2d(robotYaw) < 0)
      {
          if (angle2GlobalGoal < 0)
          {
              rotval = (180 + angle2GlobalGoal) + (180 - rad2d(robotYaw));
          }
          else
          {
              rotval = -((180 - angle2GlobalGoal) + (180 + rad2d(robotYaw)));
          }
      }
      else
      {
          rotval = angle2GlobalGoal - rad2d(robotYaw);
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
      ROS_WARN("Rotation value: %.2f", rotval);
      if (validateRotInPlace())
      {
            aprox_rot = rotationInPlace(d2rad(rotval), 5);
            if (!aprox_rot)
            {
                aproximated = true;
                setGoalReachedFlag(1);
                ROS_WARN("Aproximated");
            }
      }
      else
      {
        aproximated = true;
        setGoalReachedFlag(1);
        ROS_WARN("Arrived but not aproximated");
      }
    }
  }
} // namespace Navigators

void ArcoPathTracker::navigate()
{
    if (navigate_server_ptr->isActive() || navigate_server_ptr->isPreemptRequested() )
    {
        computeGeometry();

        moveNonHolon();
        publishCmdVel();
    }
    if (navigate_server_ptr->isPreemptRequested())
   {
      navigatePreemptCallback();
   }
}

void ArcoPathTracker::publishCmdVel()
{
  if (1)
  {
      if (navigationPaused)
          navigationPaused = false;

      fillFeedback(Vx, Vy, Wz, false, "ok");
      vel.angular.z = Wz;
      vel.linear.x = Vx;
      vel.linear.y = Vy;
      twistPub.publish(vel);
  }
  else if (!navigationPaused)
  {
      fillFeedback(0, 0, 0, true, "Security stop");
    printf("Entro 2\n");
      
      publishZeroVel();
      navigationPaused = true;
  }
}

/**
 * Exponential speed calculator
 * @max: speed at inf
 * @exp_const: the decay constant
 * @var: the variable to be function of
 * v=max*(1-exp(-exp_cons*var))
**/
inline float ArcoPathTracker::getVel(float max, float exp_const, float var)
{
    return max * (1 - exp(-exp_const * fabs(var))) * var / fabs(var);
}
    
inline float ArcoPathTracker::getVel(double max, double exp_const, double var)
{
    return max * (1 - exp(-exp_const * fabs(var))) * var / fabs(var);
}
/**
 *  Aux function to get an angle in radians and viceversa
 *  @angle to convert to radians
**/

inline double ArcoPathTracker::euclideanDistance(double x0, double y0, double x, double y)
{
    return sqrt(pow(x - x0, 2) + pow(y - y0, 2));
}
inline double ArcoPathTracker::euclideanDistance(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
{
    return euclideanDistance(pose1.pose.position.x, pose1.pose.position.y, pose2.pose.position.x, pose2.pose.position.y);
}
inline double ArcoPathTracker::euclideanDistance(geometry_msgs::PoseStamped next)
{
    return euclideanDistance(0, 0, next.pose.position.x, next.pose.position.y);
}


void ArcoPathTracker::publishZeroVel()
{
    Vx = 0;
    Vy = 0;
    Wz = 0;

    vel.angular.z = Vx;
    vel.linear.x = Vy;
    vel.linear.y = Wz;

    twistPub.publish(vel);
}

void ArcoPathTracker::computeGeometry()
{
  nextPoseBlFrame = transformPose(traj_marsupial_, global_frame_id, robot_frame);
  globalGoalBlFrame = transformPose(globalGoal, global_frame_id, robot_frame);
  angle2GlobalGoal = getYawFromQuat(globalGoal.pose.orientation);
  dist2GlobalGoal = euclideanDistance(globalGoalBlFrame);
  angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);
  dist2NextPoint = euclideanDistance(nextPoseBlFrame);
}

bool ArcoPathTracker::rotationInPlace(geometry_msgs::Quaternion finalOrientation, double threshold_)
{
    static geometry_msgs::PoseStamped robotPose;
    static tf2::Quaternion finalQ, robotQ;

    robotPose.header.frame_id = robot_frame;
    robotPose.header.stamp = ros::Time(0);
    robotPose.pose.orientation.w = 1;
    robotPose = transformPose(robotPose, robot_frame, global_frame_id);

    robotQ.setW(robotPose.pose.orientation.w);
    robotQ.setZ(robotPose.pose.orientation.z);

    finalQ.setW(finalOrientation.w);
    finalQ.setZ(finalOrientation.z);

    //This give us the angular difference to the final orientation
    tf2Scalar shortest = tf2::angleShortestPath(robotQ, finalQ);
    double sh = static_cast<double>(shortest);
    cout << "Shortest: " << sh << endl;

    return rotationInPlace(shortest, threshold_);
}

bool ArcoPathTracker::rotationInPlace(tf2Scalar dYaw, double threshold_)
{
    static tf2::Quaternion q_rot, q_f, robotQ;
    static geometry_msgs::PoseStamped robotPose;
    robotPose.header.frame_id = robot_frame;
    robotPose.header.stamp = ros::Time(0);
    robotPose.pose.orientation.w = 1;
    robotPose = transformPose(robotPose, robot_frame, global_frame_id);

    robotQ.setW(robotPose.pose.orientation.w);
    robotQ.setZ(robotPose.pose.orientation.z);

    q_rot.setRPY(0, 0, dYaw);

    q_f = robotQ * q_rot;
    q_f.normalize();

    static double var;
    var = static_cast<double>(dYaw); //radians
    cout << "Rotation var: " << rad2d(var) << endl;
    if (fabs(rad2d(var)) > threshold_)
    {
        
        Wz = getVel(angMaxSpeed, a, var);
        std::cout << "Wz= " << Wz << " , angMaxSpeed= " << angMaxSpeed << " , a= " << a << " , var=" << var << std::endl;;
        return true;
    }
    else
    {
        Wz = 0;
        return false;
    }
}

float ArcoPathTracker::getYawFromQuat(geometry_msgs::Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y / M_PI * 180;
}

void ArcoPathTracker::fillFeedback(double vx, double vy, double wz, bool sec_stop, std::string text_)
{
    navFb.header.stamp = ros::Time::now();
    navFb.header.seq++;
    navFb.feedback.distance_to_goal.data = dist2GlobalGoal;
    navFb.feedback.speed.x = vx;
    navFb.feedback.speed.y = vy;
    navFb.feedback.speed.z = wz;
    navFb.feedback.security_stop = sec_stop;
    navFb.status.text = text_;
    navigate_server_ptr->publishFeedback(navFb.feedback);
}

//Okey, this callback is reached when publishing over Navigate3D/cancel topic a empty message
void ArcoPathTracker::navigatePreemptCallback()
{
  navResult.arrived = false;
  navResult.finalDist.data = sqrt(z_old*z_old+y_old*y_old+x_old*x_old);//TODO: Put the real one
  navigate_server_ptr->setPreempted(navResult,"Navigation Goal Preempted Call received");
}

void ArcoPathTracker::navigateGoalCallback()
{
 if (navigate_server_ptr->isNewGoalAvailable())
    {
        ROS_INFO("Accepting new goal ");
        navGoal = navigate_server_ptr->acceptNewGoal();
        globalGoal.pose = navGoal->global_goal;
        cout << "globalGoal: " << globalGoal.pose.position.x << " " << globalGoal.pose.position.y << " " << globalGoal.pose.orientation.z << " " << globalGoal.pose.orientation.w << endl;

        traj_marsupial_.transforms.resize(1);
        traj_marsupial_.velocities.resize(1);
        traj_marsupial_.accelerations.resize(1);
        traj_marsupial_.transforms[0].translation.x = globalGoal.pose.position.x;
        traj_marsupial_.transforms[0].translation.y = globalGoal.pose.position.y;
        traj_marsupial_.transforms[0].translation.z = globalGoal.pose.position.z;
        traj_marsupial_.transforms[0].rotation.x = globalGoal.pose.orientation.x;
        traj_marsupial_.transforms[0].rotation.y = globalGoal.pose.orientation.y;
        traj_marsupial_.transforms[0].rotation.z = globalGoal.pose.orientation.z;
        traj_marsupial_.transforms[0].rotation.w = globalGoal.pose.orientation.w;
        traj_marsupial_.velocities[0].linear.x = 0.0;
        traj_marsupial_.velocities[0].linear.y = 0.0;
        traj_marsupial_.velocities[0].linear.z = 0.0;
        traj_marsupial_.accelerations[0].linear.x = 0.0;
        traj_marsupial_.accelerations[0].linear.y = 0.0;
        traj_marsupial_.accelerations[0].linear.z = 0.0;
        traj_marsupial_.time_from_start = ros::Duration(0.5);

        time_count = ros::Time::now();
        setGoalReachedFlag(0);
    }
}

void ArcoPathTracker::setGoalReachedFlag(bool status_)
{
    if (status_ && !navResult.arrived)
    {
        navResult.arrived = true;
        //TODO Fill these fields correctly
        navResult.finalAngle.data = angle2GlobalGoal;
        navResult.finalDist.data = dist2GlobalGoal;
        navigate_server_ptr->setSucceeded(navResult, "Goal reached succesfully");
        trajReceived = false;
        printf("Entro 1\n");

        publishZeroVel();
        ROS_WARN("Arrived");
    }
    else if (!status_ && navResult.arrived)
    {
        navResult.arrived = false;
        backwards = false;
        aproximated = false;
        phase2 = true;
        phase1 = true;
    }
}

geometry_msgs::PoseStamped ArcoPathTracker::transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = from;
    pose.header.seq = rand();
    pose.header.stamp = ros::Time::now();

    pose.pose.orientation = point.transforms[0].rotation;

    pose.pose.position.x = point.transforms[0].translation.x;
    pose.pose.position.y = point.transforms[0].translation.y;
    pose.pose.position.z = point.transforms[0].translation.z;
    return transformPose(pose, from, to);
}

geometry_msgs::PoseStamped ArcoPathTracker::transformPose(geometry_msgs::PoseStamped originalPose, std::string from, std::string to)
{
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped nextPoseStamped;

    try
    {
        transformStamped = tfBuffer->lookupTransform(to, from, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("No transform %s", ex.what());
    }

    tf2::doTransform(originalPose, nextPoseStamped, transformStamped);

    return nextPoseStamped;
}

