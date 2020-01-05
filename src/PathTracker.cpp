#include <navigator/PathTracker.hpp>

void PathTracker::dynReconfCb(arco_path_tracker::PathTrackerConfig &config, uint32_t level)
{
    this->angle1 = config.angle1;
    this->angle2 = config.angle2;
    this->angle3 = config.angle3;

    this->holon = config.holonomic;
    this->doNavigate = config.do_navigate;
    this->angMaxSpeed = config.angular_max_speed;
    this->linMaxSpeed = config.linear_max_speed;
    this->distMargin = config.dist_margin;
    this->a = config.a;
    this->b = config.b;
    this->bBack = config.b_back;
    this->linMaxSpeedBack = config.linear_max_speed_back;
}
PathTracker::PathTracker()
{

    //Pointer to the security margin object createdÃ§
    nh.reset(new ros::NodeHandle("~"));
    margin.reset(new SecurityMargin);
    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));

    server.reset(new dynamic_reconfigure::Server<arco_path_tracker::PathTrackerConfig>);
    f.reset(new dynamic_reconfigure::Server<arco_path_tracker::PathTrackerConfig>::CallbackType);

    *f = boost::bind(&PathTracker::dynReconfCb, this, _1, _2);
    server->setCallback(*f);
    costmap_clean_srv = nh->serviceClient<std_srvs::Trigger>("/custom_costmap_node/reset_costmap");
    
    nh->param("debug", debug, (bool)true);
    nh->param("do_navigate", doNavigate, (bool)true);
    nh->param("holonomic", holon, (bool)true);
    nh->param("angular_max_speed", angMaxSpeed, (double)0.4);
    nh->param("linear_max_speed", linMaxSpeed, (double)0.2);
    nh->param("ramp_speed", ramp_speed, (double)0.1);
    nh->param("linear_max_speed_back", linMaxSpeedBack, (double)0.2);
    nh->param("angle_margin", angleMargin, (double)10);
    nh->param("dist_margin", distMargin, (double)0.35);
    nh->param("a", a, (double)0.5);
    nh->param("b", b, (double)0.5);
    nh->param("b_back", bBack, (double)0.5);
    nh->param("start_orientate_dist", orientDist, (double)0.5);
    nh->param("robot_base_frame", robot_frame, (string) "base_link");
    nh->param("world_frame", world_frame, (string) "map");

    nh->param("angle1", angle1, (double)20);
    nh->param("angle2", angle2, (double)65);
    nh->param("angle3", angle3, (double)15);
    nh->param("/ns_ugv", ns_ugv, (std::string) "");
    nh->param("rot_thresh", rot_thresh, (int) 350);
    nh->param("dist_aprox1", dist_aprox1,(double)0.05);

    //Publishers, only twist and markers
    twistPub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    markersPub = nh->advertise<visualization_msgs::Marker>("speedMarker", 2);

    localPathSub = nh->subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/local_planner_node/local_path", 2, &PathTracker::localPathCb, this);
    missionFbSub = nh->subscribe<upo_actions::ExecuteMissionActionFeedback>("/Execute_Mission/feedback", 1, &PathTracker::missionFbCb, this);
    //Navigate action server configuration
    std::string server_name = "/" + ns_ugv + "/Navigation";
    navigate_server_ptr.reset(new NavigateServer(*nh, server_name.c_str(), false));
    navigate_server_ptr->registerGoalCallback(boost::bind(&PathTracker::navGoalCb, this));
    navigate_server_ptr->registerPreemptCallback(boost::bind(&PathTracker::navPreemptCb, this));
    navigate_server_ptr->start();

    //Rotation action server configuration
    rot_server_ptr.reset(new RotationInPlaceServer(*nh, "/Recovery_Rotation", false));
    rot_server_ptr->registerGoalCallback(boost::bind(&PathTracker::rotGoalCb, this));
    rot_server_ptr->registerPreemptCallback(boost::bind(&PathTracker::rotPreemptCb, this));
    rot_server_ptr->start();

    //Service to check if it's possible a rotation in place consulting the costmap
    check_rot_srv = nh->serviceClient<theta_star_2d::checkObstacles>("/custom_costmap_node/check_env");

    backwards = false;
    recoveryRotation = false;
    aproximated = false;
    phase2 = true;
    phase1 = true;
    //Flags for internal states
    trajReceived = false;
    rampMode=false;
    //last_trj_stamp = ros::Time::now();
    Vx = Vy = Wz = 0;
    //Configure speed direction marker
    configureMarkers();
}
void PathTracker::missionFbCb(const upo_actions::ExecuteMissionActionFeedbackConstPtr &fb)
{
    if (fb->feedback.waypoint_type.data == RAMP_END)
    {
        ROS_WARN("START RAMP");
        rampMode = true;
    }
    else if(rampMode)
    {
        ROS_WARN("FINISH RAMP");
        rampMode = false;
    }

}
void PathTracker::computeGeometry()
{
    nextPoseBlFrame = transformPose(nextPoint, world_frame, robot_frame);
    globalGoalBlFrame = transformPose(globalGoal, world_frame, robot_frame);
    angle2GlobalGoal = getYawFromQuat(globalGoal.pose.orientation);
    dist2GlobalGoal = euclideanDistance(globalGoalBlFrame);
    angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);
    dist2NextPoint = euclideanDistance(nextPoseBlFrame);
}
bool PathTracker::checkPathTimeout()
{
    bool ret = false;
    if ( navigate_server_ptr->isActive() && !trajReceived || ros::Time::now() - last_trj_stamp > ros::Duration(2))
    {
        ROS_INFO("PATH TIMEOUT");
        publishZeroVel();
        ret = true;
    }
    return ret;
} // namespace Navigators
void PathTracker::navigate()
{
    if (rot_server_ptr->isNewGoalAvailable())
    {
        rot_inplace = rot_server_ptr->acceptNewGoal();
    }

    if (navigate_server_ptr->isNewGoalAvailable())
    {
        ROS_INFO("Accepting new goal ");
        navigate_goal = navigate_server_ptr->acceptNewGoal();
        globalGoal.pose = navigate_goal->global_goal;
        cout << "globalGoal: " << globalGoal.pose.position.x << " " << globalGoal.pose.position.y << " " << globalGoal.pose.orientation.z << " " << globalGoal.pose.orientation.w << endl;

        time_count = ros::Time::now();
        setGoalReachedFlag(0);
    }
    if (!checkPathTimeout() && (navigate_server_ptr->isActive() || !aproximated))
    {
        computeGeometry();

        if (dist2GlobalGoal < 1)
        {
            margin->setMode(1);
        }
        if(!rampMode){
            moveNonHolon();
        }else{
            moveForward();
        }

        publishCmdVel();
    }
}
void PathTracker::moveForward(){
    Vx=ramp_speed;
    Wz=0;
    if(dist2GlobalGoal < 0.2){
         aproximated = true;
        setGoalReachedFlag(1);
    }else{
        publishCmdVel();
    }
}
void PathTracker::publishCmdVel()
{
    if (1) //margin->canIMove())
    {
        if (navigationPaused)
        {
            navigationPaused = false;
        }
        fillFeedback(Vx, Vy, Wz, false);

        vel.angular.z = Wz;
        vel.linear.x = Vx;
        vel.linear.y = Vy;

        twistPub.publish(vel);
        publishMarkers();
    }
    else if (!navigationPaused)
    {
        fillFeedback(0, 0, 0, true, "Security stop");
        publishZeroVel();
        navigationPaused = true;
    }
}
//TODO Implement
bool PathTracker::validateRotInPlace(int thresh)
{
    bool ret = false;
    
    theta_star_2d::checkObstacles trg;
   
    trg.request.thresh.data=thresh;

    check_rot_srv.call(trg);
    ROS_INFO("Validate rotation requested: %s", trg.response.message.c_str());
    if (trg.response.success)
        return true;

    return ret;
}
void PathTracker::moveNonHolon()
{
    if (angle2NextPoint < 0)
    {
        angleBack = angle2NextPoint + M_PI;
    }
    else
    {
        angleBack = angle2NextPoint - M_PI;
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
                rotationInPlace(angle2NextPoint, 0, true);
                Vx = 0;
            }
            else if (!backwards)
            {
                ROS_INFO("Enabling backwards");
                backwards = true;
                time_count = ros::Time::now();
            }
            else if (ros::Time::now() - time_count > ros::Duration(100) && validateRotInPlace()) //Reset backwards
            {
                ROS_INFO("\t 2");
                backwards = false;
            }
            else if (fabs(angleBack) > fabs(d2rad(angle3))) //Too much angular distance, do only rotation in place
            {
                ROS_INFO("\t 3");
                rotationInPlace(angleBack, 0, true);
                Vx = 0;
            }
            else
            {
                ROS_INFO("\t 4");
                Vx = -getVel(linMaxSpeedBack, bBack , dist2GlobalGoal);
                rotationInPlace(angleBack, 0, false);
            }
        }
        else
        {
            // ROS_INFO("\t 5");
            Vx = getVel(linMaxSpeed, b, dist2GlobalGoal);
            Vy = 0;
            rotationInPlace(angle2NextPoint, 0, false);
        }
    }
    else if (!aproximated)
    {

        if (phase1){
            phase1 = false;
            std_srvs::Trigger trg;
            costmap_clean_srv.call(trg);
        }
        //setGoalReachedFlag(1);
        if (phase2) //&& !rotationInPlace(angle2GlobalGoal, 5))
        {   
            Wz=0;
            ROS_INFO("Fase 1");
            
            if (globalGoalBlFrame.pose.position.x > 0 && globalGoalBlFrame.pose.position.x < dist_aprox1)
            {
                Vx = 0.01;
            }
            else if (globalGoalBlFrame.pose.position.x < 0 && globalGoalBlFrame.pose.position.x > -dist_aprox1)
            {
                Vx = -0.01;
            }
            else
            {
                ROS_INFO("OUT FASE 1");
                phase2 = false;
                Vx = 0;
            }
            //Vx = getVel(globalGoalBlFrame.pose.position.x < 0 ? -linMaxSpeed : linMaxSpeed, b, dist2GlobalGoal);
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
            robotPose = transformPose(robotPose, robot_frame, world_frame);

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
            ROS_INFO("Rotation value: %.2f", rotval);
            if (validateRotInPlace(rot_thresh))
            {
                aprox_rot = rotationInPlace(d2rad(rotval), 5, true);
                if (!aprox_rot)
                {
                    aproximated = true;
                    setGoalReachedFlag(1);
                    ROS_INFO("Aproximated");
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
void PathTracker::moveHolon(double finalYaw)
{

    double v = getVel(linMaxSpeed, b, dist2GlobalGoal);

    Vx = cos(angle2NextPoint) * v;
    Vy = sin(angle2NextPoint) * v;

    Wz = getVel(angMaxSpeed, 4 * a, angle2NextPoint);
    if (fabs(angle2NextPoint) > M_PI_2)
    {
        Vx /= 1.5;
        Vy /= 1.5;
    }
    else if (fabs(angle2NextPoint) > M_PI_4)
    {
        Vx /= 1.2;
        Vy /= 1.2;
    }

    if (dist2GlobalGoal < orientDist)
        setRobotOrientation(finalYaw, 0, 0, angMaxSpeed, 0);
}

bool PathTracker::rotationInPlace(geometry_msgs::Quaternion finalOrientation, double threshold_, bool final = false)
{
    static geometry_msgs::PoseStamped robotPose;
    static tf2::Quaternion finalQ, robotQ;

    robotPose.header.frame_id = robot_frame;
    robotPose.header.stamp = ros::Time(0);
    robotPose.pose.orientation.w = 1;
    robotPose = transformPose(robotPose, robot_frame, world_frame);

    robotQ.setW(robotPose.pose.orientation.w);
    robotQ.setZ(robotPose.pose.orientation.z);

    finalQ.setW(finalOrientation.w);
    finalQ.setZ(finalOrientation.z);

    //This give us the angular difference to the final orientation
    tf2Scalar shortest = tf2::angleShortestPath(robotQ, finalQ);
    double sh = static_cast<double>(shortest);
    cout << "Shortest: " << sh << endl;

    return rotationInPlace(shortest, threshold_, final);
}
bool PathTracker::rotationInPlace(tf2Scalar dYaw, double threshold_, bool final = false)
{
    static tf2::Quaternion q_rot, q_f, robotQ;
    static geometry_msgs::PoseStamped robotPose;
    robotPose.header.frame_id = robot_frame;
    robotPose.header.stamp = ros::Time(0);
    robotPose.pose.orientation.w = 1;
    robotPose = transformPose(robotPose, robot_frame, world_frame);

    robotQ.setW(robotPose.pose.orientation.w);
    robotQ.setZ(robotPose.pose.orientation.z);

    q_rot.setRPY(0, 0, dYaw);

    q_f = robotQ * q_rot;
    q_f.normalize();

    /*static double angle2;
    angle2 = tf2::angleShortestPath(robotQ, q_f);

    if (angle2 > dYaw)
    {
        dYaw *= -1;
    }*/

    static double var;
    var = static_cast<double>(dYaw); //radians
    // cout << "Rotation var: " << rad2d(var) << endl;
    if (fabs(rad2d(var)) > threshold_)
    {
        if(rad2d(var) > 90){
            var=M_PI_2;
        }else if(rad2d(var) < -90){
            var=-M_PI_2;
        }
        
        Wz = getVel(final?angMaxSpeed+0.05:angMaxSpeed, final ? a / 2 : a, var); //TODO poner el 0.2 como parametro

        return true;
    }
    else
    {
        Wz = 0;
        return false;
    }
}

void PathTracker::setRobotOrientation(float finalYaw, bool goal, bool pub, float speed, float angleMargin_)
{
    //We receive the quaternion q in the frame map. We have to know the orientation of the robot
    //also in the map frame

    geometry_msgs::PoseStamped robotPose;
    robotPose.header.frame_id = robot_frame;
    robotPose.header.stamp = ros::Time(0);
    robotPose.header.seq = rand();

    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;

    robotPose.pose.orientation.w = 1;

    robotPose = transformPose(robotPose, robot_frame, world_frame);
    float robotYaw = getYawFromQuat(robotPose.pose.orientation);
    if (robotYaw < 0)
        robotYaw += 360;
    if (finalYaw < 0)
        finalYaw += 360;

    float yawDif = finalYaw - robotYaw;

    if (fabs(yawDif) > 180)
        yawDif -= 360 * yawDif / fabs(yawDif);

    if (fabs(yawDif) > angleMargin_)
    {
        //This one works VERY WELL DONT TOUCH IT, antes era 2 en vez de 3
        Wz = 3 * (yawDif * (speed - 0.1) / (180 - angleMargin_) + speed - 180 / (180 - angleMargin_) * (speed - 0.1));
    }
    else if (goal)
    {
        setGoalReachedFlag(1);
    }

    if (pub)
        publishCmdVel();
}

void PathTracker::setGoalReachedFlag(bool status_)
{
    if (status_ && !navigate_result.arrived)
    {
        navigate_result.arrived = true;

        //TODO Fill these fields correctly
        navigate_result.finalAngle.data = angle2GlobalGoal;
        navigate_result.finalDist.data = dist2GlobalGoal;
        navigate_server_ptr->setSucceeded(navigate_result, "Goal reached succesfully");

        trajReceived = false;
        publishZeroVel();
        ROS_WARN("Arrived");
    }
    else if (!status_ && navigate_result.arrived)
    {
        navigate_result.arrived = false;
        backwards = false;
        aproximated = false;
        phase2 = true;
        phase1 = true;
    }
}
void PathTracker::fillFeedback(double vx, double vy, double wz, bool sec_stop, std::string text_)
{
    navigate_fb.header.stamp = ros::Time::now();
    navigate_fb.header.seq++;
    navigate_fb.feedback.distance_to_goal.data = dist2GlobalGoal;
    navigate_fb.feedback.speed.x = vx;
    navigate_fb.feedback.speed.y = vy;
    navigate_fb.feedback.speed.z = wz;
    navigate_fb.feedback.security_stop = sec_stop;
    navigate_fb.status.text = text_;
    navigate_server_ptr->publishFeedback(navigate_fb.feedback);
}
/*
*   Configuration functions
*/
void PathTracker::configureMarkers()
{
    markers.resize(2);
    //Linear Speed markers
    markers.at(0).header.frame_id = robot_frame;
    markers.at(0).header.stamp = ros::Time();
    markers.at(0).ns = "path_tracker";
    markers.at(0).id = 1;
    markers.at(0).type = visualization_msgs::Marker::ARROW;
    markers.at(0).action = visualization_msgs::Marker::ADD;
    markers.at(0).lifetime = ros::Duration(1);
    markers.at(0).scale.y = 0.1;
    markers.at(0).scale.z = 0.2;
    markers.at(0).pose.position.x = 0;
    markers.at(0).pose.position.y = 0;
    markers.at(0).pose.position.z = 1;
    markers.at(0).color.a = 1.0;
    markers.at(0).color.b = 1.0;
    markers.at(0).color.g = 1.0;
    markers.at(0).color.r = 0.0;
    //Angular Speed Arrow
    markers.at(1).header.frame_id = robot_frame;
    markers.at(1).header.stamp = ros::Time();
    markers.at(1).ns = "path_tracker";
    markers.at(1).id = 2;
    markers.at(1).type = visualization_msgs::Marker::ARROW;
    markers.at(1).action = visualization_msgs::Marker::ADD;
    markers.at(1).lifetime = ros::Duration(1);
    markers.at(1).scale.y = 0.1;
    markers.at(1).scale.z = 0.2;
    markers.at(1).pose.position.x = 0;
    markers.at(1).pose.position.y = 0;
    markers.at(1).pose.position.z = 1;
    markers.at(1).color.a = 1.0;
    markers.at(1).color.b = 1.0;
    markers.at(1).color.g = 0.0;
    markers.at(1).color.r = 1.0;

    tf2::Quaternion quat;
    quat.setRPY(0, M_PI_2, 0);
    markers.at(1).pose.orientation.x = quat.getX();
    markers.at(1).pose.orientation.y = quat.getY();
    markers.at(1).pose.orientation.z = quat.getZ();
    markers.at(1).pose.orientation.w = quat.getW();
}
void PathTracker::publishMarkers()
{
    markers.at(0).scale.x = 2 * sqrtf(Vx * Vx + Vy * Vy);

    static tf2::Quaternion quat;
    static float yaw;

    yaw = atan2(Vy, Vx);
    quat.setRPY(0, 0, yaw); // Create this quatern

    markers.at(0).pose.orientation.z = quat.getZ();
    markers.at(0).pose.orientation.w = quat.getW();
    markers.at(1).scale.x = -2 * Wz; //Minus sign is to follow right hand rule

    markersPub.publish(markers.at(0));
    markersPub.publish(markers.at(1));
}
/*
*   Callbacks and related functions
*/
void PathTracker::localPathCb(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg)
{
    if (navigate_server_ptr->isActive())
    {
        nextPoint = msg->points[msg->points.size() > 1 ? 1 : 0];
        trajReceived = true;
        last_trj_stamp = msg->header.stamp;
        margin->setMode(0);
    }
}
void PathTracker::publishZeroVel()
{
    Vx = 0;
    Vy = 0;
    Wz = 0;

    vel.angular.z = Vx;
    vel.linear.x = Vy;
    vel.linear.y = Wz;

    publishMarkers();
    //ROS_INFO("Publishing zero vel");
    twistPub.publish(vel);
}
void PathTracker::navGoalCb()
{
}
void PathTracker::navPreemptCb()
{
    navigate_server_ptr->setPreempted();
    backwards = false;
    recoveryRotation = false;
    aproximated = false;
    phase2 = true;
    phase1 = true;
    //Flags for internal states
    trajReceived = false;
    //last_trj_stamp = ros::Time::now();
    Vx = Vy = Wz = 0;
}
void PathTracker::rotGoalCb()
{
}
void PathTracker::rotPreemptCb()
{
}
//Aux Functions

geometry_msgs::PoseStamped PathTracker::transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to)
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
geometry_msgs::PoseStamped PathTracker::transformPose(geometry_msgs::PoseStamped originalPose, std::string from, std::string to)
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
float PathTracker::getYawFromQuat(geometry_msgs::Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y / M_PI * 180;
}
