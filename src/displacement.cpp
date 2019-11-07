/*
* Navigator Class: For holonomic and no holonomic robots
* Rafael Rey Arcenegui 2019, UPO
*/
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>

namespace Navigators
{
bool Displacement::activateBackwardSrv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &rep)
{
    if (debug)
    {
        debug = false;
    }
    else
    {
        debug = true;
    }

    return true;
}
void Displacement::refreshParams()
{
    ros::param::get("/nav_node/holonomic", holonomic);
    ros::param::get("/nav_node/do_navigate", do_navigate);
    ros::param::get("/nav_node/angular_max_speed", angularMaxSpeed);
    ros::param::get("/nav_node/linear_max_speed", linearMaxSpeed);
    ros::param::get("/nav_node/angle_margin", angleMargin);
    ros::param::get("/nav_node/dist_margin", distMargin);
    ros::param::get("/nav_node/a", a);
    ros::param::get("/nav_node/b", b);
    ros::param::get("/nav_node/start_orientate_dist", startOrientateDist);

    //The idea is to save the smallest distance to anyone and change the value of b according to this distance
    closest.second.first = std::numeric_limits<float>::max();

    for (auto it = dist2people.begin(); it != dist2people.end(); ++it)
        if (it->second.first < closest.second.first)
            closest.second = it->second;

    //Now we set 1.5 has a distance to take into account persons
    //TODO: Tener en cuenta solo las personas que entran dentro de un cono apuntando en la dirección de movimiento

    if (closest.second.first < 1.5)
    {
        b = old_b * closest.second.first / 1.5;
    }
}
void Displacement::laser1Callback(const sensor_msgs::LaserScanConstPtr &scan)
{
    scanL = scan;
    scanRGot = true;
}
void Displacement::laser2Callback(const sensor_msgs::LaserScanConstPtr &scan)
{
    scanR = scan;
    scanRGot = true;
}
//Displacement Class functions
Displacement::Displacement(ros::NodeHandle *n, tf2_ros::Buffer *tfBuffer_)
{
    nh = n;
    //Pointer to the security margin object createdÃ§
    margin.setParams(n);

    tfBuffer = tfBuffer_;

    //? Detect if possible to rotate in place
    scanRGot = false, scanLGot = false, backwards = false;
    laser1_sub = nh->subscribe<sensor_msgs::LaserScan>("/scanLeft", 1, &Displacement::laser1Callback, this);
    laser2_sub = nh->subscribe<sensor_msgs::LaserScan>("/scanRight", 1, &Displacement::laser2Callback, this);
    //?
    //Right now we will put the ARCO cmd vel topic but in the future it will selectable
    twist_pub = nh->advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);
    moving_state_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/muving_state", 1);
    speed_marker_pub = nh->advertise<visualization_msgs::Marker>("/speed_marker", 1);
    dist2goal_pub = nh->advertise<std_msgs::Float32>("/dist2goal", 0);
    dist2goal.data = 0;

    approach_man_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/aproach_manoeuvre", 1);
    rot_recovery_status_pub = nh->advertise<std_msgs::Bool>("/nav_node/rot_recovery_status", 1);

    navigate_server_ptr.reset(new NavigateServer(*nh, "Navigation", false));
    navigate_server_ptr->registerGoalCallback(boost::bind(&Displacement::navGoalCb, this));
    navigate_server_ptr->registerPreemptCallback(boost::bind(&Displacement::navPreemptCb, this));
    navigate_server_ptr->start();

    rot_server_ptr.reset(new RotationInPlaceServer(*nh, "Recovery_Rotation", false));
    rot_server_ptr->registerGoalCallback(boost::bind(&Displacement::rotGoalCb, this));
    rot_server_ptr->registerPreemptCallback(boost::bind(&Displacement::rotPreemptCb, this));
    rot_server_ptr->start();

    backwardsServer = nh->advertiseService("backwards_on", &Displacement::activateBackwardSrv, this);

    nh->param("/nav_node/debug", debug, (bool)true);
    nh->param("/nav_node/do_navigate", do_navigate, (bool)true);
    nh->param("/nav_node/holonomic", holonomic, (bool)true);
    nh->param("/nav_node/angular_max_speed", angularMaxSpeed, (float)0.5);
    nh->param("/nav_node/linear_max_speed", linearMaxSpeed, (float)0.3);
    nh->param("/nav_node/angle_margin", angleMargin, (float)10);
    nh->param("/nav_node/dist_margin", distMargin, (float)0.35);
    nh->param("/nav_node/a", a, (float)5);
    nh->param("/nav_node/b", b, (float)5);
    nh->param("/nav_node/start_orientate_dist", startOrientateDist, (float)0.5);
    nh->param("/nav_node/robot_base_frame", robot_frame, (string) "base_link");
    nh->param("/nav_node/world_frame", world_frame, (string) "map");
    old_b = b;

    rotation_fb.header.frame_id = world_frame;
    navigate_fb.header.frame_id = world_frame;
    //Start flags values
    //Flags to publish

    rot.data = false;
    goalReached.data = false;
    recoveryRotation = false;

    //Flags for internal states
    trajReceived = false;

    alpha = 30;
    Vx = Vy = Wz = 0;
    //Configure speed direction marker

    speed.header.frame_id = robot_frame;
    speed.header.stamp = ros::Time();
    speed.ns = "path_tracker";
    speed.id = 1;
    speed.type = visualization_msgs::Marker::ARROW;
    speed.action = visualization_msgs::Marker::ADD;
    speed.lifetime = ros::Duration(1);
    speed.scale.y = 0.1;
    speed.scale.z = 0.2;
    speed.pose.position.x = 0;
    speed.pose.position.y = 0;
    speed.pose.position.z = 1;
    speed.color.a = 1.0;
    speed.color.b = 1.0;
    speed.color.g = 1.0;
    speed.color.r = 0.0;

    rot_speed.header.frame_id = robot_frame;
    rot_speed.header.stamp = ros::Time();
    rot_speed.ns = "path_tracker";
    rot_speed.id = 2;
    rot_speed.type = visualization_msgs::Marker::ARROW;
    rot_speed.action = visualization_msgs::Marker::ADD;
    rot_speed.lifetime = ros::Duration(1);
    rot_speed.scale.y = 0.1;
    rot_speed.scale.z = 0.2;
    rot_speed.pose.position.x = 0;
    rot_speed.pose.position.y = 0;
    rot_speed.pose.position.z = 1;
    rot_speed.color.a = 1.0;
    rot_speed.color.b = 1.0;
    rot_speed.color.g = 0.0;
    rot_speed.color.r = 1.0;

    tf2::Quaternion quat;
    quat.setRPY(0, M_PI_2, 0);
    rot_speed.pose.orientation.x = quat.getX();
    rot_speed.pose.orientation.y = quat.getY();
    rot_speed.pose.orientation.z = quat.getZ();
    rot_speed.pose.orientation.w = quat.getW();
}
void Displacement::rotGoalCb()
{
    bool suc = rotateToRefresh();

    //This flag topic was used by the leds node
    rot.data = true;
    rot_recovery_status_pub.publish(rot);
}
void Displacement::rotPreemptCb()
{
    recoveryRotation = false;
}
void Displacement::navGoalCb()
{
}
void Displacement::navPreemptCb()
{
}
bool Displacement::validateRotInPlace()
{
    bool ret = true;

    scanRGot = false;
    scanLGot = false;
    /*while (!scanRGot && !scanLGot)
    {
        ros::spinOnce();
    }
    //Okey, now you have the two latests scans
    int cnt1 = 0, cnt2 = 0;
    for (auto it = scanR->ranges.cbegin(); it != scanR->ranges.cend(); ++it)
    {
        if (*it < 0.5)
            ++cnt1;
    }
    for (auto it = scanL->ranges.cbegin(); it != scanL->ranges.cend(); ++it)
    {
        if (*it < 0.5)
            ++cnt2;
    }

    if (cnt1 > 5 || cnt2 > 5)
        ret = false;
    */
    return ret;
}
bool Displacement::rotateToRefresh()
{
    recoveryRotation = true;

    PoseStamp bl_pose;

    bl_pose.pose.position.x = 0;
    bl_pose.pose.position.y = 0;

    bl_pose.pose.orientation.w = 1;
    bl_pose.pose.orientation.z = 0;

    start_pose = transformPose(bl_pose, robot_frame, world_frame);

    return true;
}
void Displacement::trackedPersonCb(const people_msgs::People::ConstPtr &pl)
{
    peopl = *pl;
    computeDistanceToPeople();
    printPeople();
}
void Displacement::trajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
    if (navigate_server_ptr->isActive())
    {
        nextPoint = trj->points[trj->points.size() > 1 ? 1 : 0];
        trajReceived = true;
        last_trj_stamp = trj->header.stamp;

        margin.setMode(0);
    }
}
void Displacement::setRobotOrientation(geometry_msgs::Quaternion q, bool goal, bool pub, float speed, float angleMargin_)
{
    setRobotOrientation(getYawFromQuat(q), goal, pub, speed, angleMargin_);
}
void Displacement::setRobotOrientation(float finalYaw, bool goal, bool pub, float speed, float angleMargin_)
{
    //We receive the quaternion q in the frame map. We have to know the orientation of the robot
    //also in the map frame

    PoseStamp robotPose;
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
        ROS_WARN("hEY");
        setGoalReachedFlag(1);
    }

    if (pub)
        publishCmdVel();
}
void Displacement::aproximateTo(geometry_msgs::PoseStamped *pose, bool isGoal, bool isHome)
{

    geometry_msgs::PoseStamped p = transformPose(*pose, world_frame, robot_frame);

    Vx = p.pose.position.x / 2;
    Vy = p.pose.position.y / 2;

    setRobotOrientation(getYawFromQuat(pose->pose.orientation), isGoal, 1, 1.3 * angularMaxSpeed, angleMargin);
}
/*
  *   The idea is to start moving in the direction of the nextPoint relative to base_link frame
  *   But, to give some preference to X direction of muvement(forward) because of reasons(in ARCO for example we have blind areas in the laterals)
  *   So we will give velocity v=(vx,vy) in the direction of the next point, but at the same time the robot will rotate to face the point and thus
  *   the mouvement will finally be in x direction. 
  *   Also, we will use differents top speed for x and y because is more dangerous to move in y direction than x. 
*/
void Displacement::moveHolon(double finalYaw)
{

    v = getVel(linearMaxSpeed, b, dist2GlobalGoal);

    Vx = cos(angle2NextPoint) * v;
    Vy = sin(angle2NextPoint) * v;

    //if (fabs(angle2NextPoint) > d2rad(angleMargin))
    //{
    Wz = getVel(angularMaxSpeed, 4 * a, angle2NextPoint);
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
    //}

    if (dist2GlobalGoal < startOrientateDist)
        setRobotOrientation(finalYaw, 0, 0, angularMaxSpeed, 0);
}
void Displacement::moveNonHolon()
{
    //TODO Change 40 value
    if (debug)
    {

        if (fabs(angle2NextPoint) > d2rad(15)) //Rot in place
        {
            if (backwards && ros::Time::now() - timeout_backwards > ros::Duration(20))
            {
                if (validateRotInPlace())
                {
                    backwards = false;
                    Wz = getVel(angularMaxSpeed, a, angle2NextPoint);
                }
            }

            if (validateRotInPlace() && !backwards)
            {
                Wz = getVel(angularMaxSpeed, a, angle2NextPoint);
            }
            else
            { //!MARCHA ATRAS
                if (!backwards)
                    timeout_backwards = ros::Time::now();

                Vx = -1 * getVel(linearMaxSpeed, b, dist2GlobalGoal);
                Vy = 0;
                Wz = -1 * getVel(angularMaxSpeed, a, angle2NextPoint);
                backwards = true;
            }
        }
        else if (!backwards)
        {
            Vx = getVel(linearMaxSpeed, b, dist2GlobalGoal);
            Vy = 0;
            Wz = getVel(angularMaxSpeed, a, angle2NextPoint);
        }
    }
    else
    {
        if (fabs(angle2NextPoint) > d2rad(15)) //Rot in place
        {
            Wz = getVel(angularMaxSpeed, a, angle2NextPoint);
        }
        else if (!backwards)
        {
            Vx = getVel(linearMaxSpeed, b, dist2GlobalGoal);
            Vy = 0;
            Wz = getVel(angularMaxSpeed, a, angle2NextPoint);
        }
    }
}
void Displacement::navigate()
{
    refreshParams();
    if (rot_server_ptr->isNewGoalAvailable())
    {
        rot_inplace = rot_server_ptr->acceptNewGoal();
    }
    if (navigate_server_ptr->isNewGoalAvailable())
    {
        navigate_goal = navigate_server_ptr->acceptNewGoal();
        globalGoal.pose = navigate_goal->global_goal;
        setGoalReachedFlag(0);
        time_count = ros::Time::now();
    }

    //Security timeout
    if (ros::Time::now() - last_trj_stamp > ros::Duration(1))
    {
        if (!timeout)
            publishZeroVelocity();

        timeout = true;
    }
    else
    {
        timeout = false;
    }
    if (recoveryRotation)
    {

        Vx = 0;
        Vy = 0;
        Wz = angularMaxSpeed;

        if (ros::Time::now() - start_pose.header.stamp > ros::Duration(2 * M_PI / Wz))
        {
            recoveryRotation = false;
            Wz = 0;
        }

        rotation_fb.header.seq++;
        rotation_fb.header.stamp = ros::Time::now();
        rotation_fb.feedback.angular_distance.data = (ros::Time::now() - start_pose.header.stamp).toSec() * Wz;
        rotation_fb.status.text = "Executing rotation in place";
        rot_server_ptr->publishFeedback(rotation_fb.feedback);
        publishCmdVel();
    }
    else if (navigate_server_ptr->isActive() && trajReceived)
    {
        if (rot.data)
        {
            rot.data = false;
            rot_recovery_status_pub.publish(rot);
        }
        Vx = 0;
        Vy = 0;
        Wz = 0;

        PoseStamp nextPoseBlFrame = transformPose(nextPoint, world_frame, robot_frame);
        angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);
        dist2NextPoint = euclideanDistance(nextPoseBlFrame);

        globalGoalPose = transformPose(globalGoal, world_frame, robot_frame);
        dist2GlobalGoal = euclideanDistance(globalGoalPose);
        angle2GlobalGoal = getYawFromQuat(globalGoalPose.pose.orientation);
        dist2goal.data = dist2GlobalGoal;
        dist2goal_pub.publish(dist2goal);

        if (dist2GlobalGoal < 1 || ros::Time::now() - time_count < ros::Duration(2))
        {
            margin.setMode(1);
            std_msgs::Bool flg;
            flg.data = true;
            approach_man_pub.publish(flg);
        }
        if (dist2GlobalGoal < distMargin && !goalReached.data)
        {
            ROS_INFO_ONCE("Maniobra de aproximacion");
            aproximateTo(&globalGoal, 1, 0);

            margin.setMode(1);
        }
        else if (holonomic)
        {
            moveHolon(getYawFromQuat(globalGoal.pose.orientation));
        }
        else
        {
            moveNonHolon();
        }
        publishCmdVel();
    }
}

/**
 * Inputs must be:
 * max: m/s or rad/s
 * var: m/s or rad
 *  
**/
float Displacement::getVel(float max, float exp_const, float var)
{
    return max * (1 - exp(-exp_const * fabs(var))) * var / fabs(var);
}
bool Displacement::hasFinished()
{
    return goalReached.data;
}
void Displacement::publishZeroVelocity()
{
    Vx = 0;
    Vy = 0;
    Wz = 0;

    vel.angular.z = Vx;
    vel.linear.x = Vy;
    vel.linear.y = Wz;
    speed.scale.x = 0;
    rot_speed.scale.x = 0;
    movingState.data = false;

    twist_pub.publish(vel);
    moving_state_pub.publish(movingState);
}
void Displacement::setGoalReachedFlag(bool status_)
{
    if (status_)
    {
        navigate_result.arrived = true;

        //TODO Fill these fields correctly
        navigate_result.finalAngle.data = angle2GlobalGoal;
        navigate_result.finalDist.data = dist2GlobalGoal;
        navigate_server_ptr->setSucceeded(navigate_result, "Goal reached succesfully");

        trajReceived = false;
        goalReached.data = true;
        publishZeroVelocity();
        ROS_WARN("Arrived");

        std_msgs::Bool flg;
        flg.data = false;
        approach_man_pub.publish(flg);
    }
    else
    {
        goalReached.data = false;
    }
}
void Displacement::publishCmdVel()
{

    movingState.data = true;

    if (Vx == 0 && Vy == 0 && Wz == 0)
        movingState.data = false;

    if (margin.canIMove() && !timeout)
    {
        if (navigationPaused)
        {
            ROS_INFO("Playing planning again");
            navigationPaused = false;
        }

        navigate_fb.header.stamp = ros::Time::now();
        navigate_fb.header.seq++;
        navigate_fb.feedback.distance_to_goal.data = dist2GlobalGoal;
        navigate_fb.feedback.speed.x = Vx;
        navigate_fb.feedback.speed.y = Vy;
        navigate_fb.feedback.speed.z = Wz;
        navigate_fb.feedback.security_stop = false;
        ROS_INFO_THROTTLE(0.5, "Sending cmd vels: Vx,Vy,Wz:\t[%.2f, %.2f]", Vx, Wz);

        vel.angular.z = Wz;
        vel.linear.x = Vx;
        vel.linear.y = Vy;
        speed.scale.x = 2 * sqrtf(Vx * Vx + Vy * Vy);

        tf2::Quaternion quat;
        float yaw = atan2(Vy, Vx);
        quat.setRPY(0, 0, yaw); // Create this quatern
        speed.pose.orientation.z = quat.getZ();
        speed.pose.orientation.w = quat.getW();

        speed_marker_pub.publish(speed);
        rot_speed.scale.x = -2 * Wz; //Minus sign is to follow right hand rule
        speed_marker_pub.publish(rot_speed);
        twist_pub.publish(vel);
        moving_state_pub.publish(movingState);
    }
    else
    {
        navigate_fb.feedback.distance_to_goal.data = dist2GlobalGoal;
        navigate_fb.feedback.speed.x = 0;
        navigate_fb.feedback.speed.y = 0;
        navigate_fb.feedback.speed.z = 0;
        navigate_fb.feedback.security_stop = true;
        navigate_fb.status.text = "Security Stop...";

        if (!navigationPaused)
        {
            publishZeroVelocity();
        }
        ROS_WARN("I cant move, pausing navigation");
        navigationPaused = true;
    }
    navigate_server_ptr->publishFeedback(navigate_fb.feedback);
}

PoseStamp Displacement::transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to)
{
    PoseStamp pose;
    pose.header.frame_id = from;
    pose.header.seq = rand();
    pose.header.stamp = ros::Time::now();

    pose.pose.orientation = point.transforms[0].rotation;

    pose.pose.position.x = point.transforms[0].translation.x;
    pose.pose.position.y = point.transforms[0].translation.y;
    pose.pose.position.z = point.transforms[0].translation.z;
    return transformPose(pose, from, to);
}
PoseStamp Displacement::transformPose(PoseStamp originalPose, std::string from, std::string to)
{

    geometry_msgs::TransformStamped transformStamped;
    PoseStamp nextPoseStamped;

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
float Displacement::getYawFromQuat(geometry_msgs::Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y / M_PI * 180;
}
void Displacement::computeDistanceToPeople()
{

    dist2people.clear();

    PoseStamp bl_pose, person;

    bl_pose.header.frame_id = robot_frame;
    bl_pose.header.seq = rand();
    bl_pose.header.stamp = ros::Time::now();

    bl_pose.pose.position.x = 0;
    bl_pose.pose.position.y = 0;
    bl_pose.pose.position.z = 0;

    bl_pose.pose.orientation.w = 1;
    bl_pose.pose.orientation.z = 0;

    bl_pose = transformPose(bl_pose, robot_frame, world_frame);
    pair<string, pair<float, float>> p;

    for (int i = 0; i < peopl.people.size(); i++)
    {

        person.pose.position = peopl.people.at(i).position;
        person = transformPose(person, world_frame, robot_frame);

        p.first = peopl.people.at(i).name;
        p.second.first = euclideanDistance(person);
        p.second.second = atan2(person.pose.position.y, person.pose.position.x) / M_PI * 180;
        if (p.second.second < 0)
            p.second.second += 360;

        dist2people.insert(p);
    }
}
void Displacement::printPeople()
{

    ROS_INFO("\x1B[32m\n\tName  \t Distance \t Angle \n");
    for (auto &it : dist2people)
    {
        ROS_INFO("\x1B[32m\t  %s\t %.2f\t %.2f\n", it.first.c_str(), it.second.first, it.second.second);
    }
    ROS_INFO("\x1B[0m");
}
//To check if anyone in the people list is in the direction mouvement cone +-40º centered in the
//velocity direction
bool Displacement::someoneInFront()
{
    return someoneInFront(Vx, Vy);
}
bool Displacement::someoneInFront(float vx, float vy)
{

    if (Vx != 0 || Vy != 0)
    {

        float theta1 = atan2(vy, vx) / M_PI * 180;
        float t2 = theta1 + alpha;
        float t3 = theta1 - alpha;

        if (theta1 < 0)
            theta1 += 360;
        if (t2 > 360)
            t2 -= 360;
        if (t3 < 0)
            t3 += 360;
        ROS_INFO_THROTTLE(1, "T2: %.2f \t T3: %.2f", t2, t3);

        //Case 1: theta 1 > alpha && theta < 360-alpha
        if (theta1 > alpha && theta1 < (360 - alpha))
        {
            for (auto &it : dist2people)
                if (it.second.second < t2 && it.second.second > t3)
                    return true;
        }
        //Case 2: theta 1 < alpha
        if (theta1 < alpha)
        {
            for (auto &it : dist2people)
            {
                if (it.second.second < t2 && (fabs(t3 - 360) + it.second.second) < 2 * alpha)
                    return true;
            }
        }
        //Case 3:  theta1 > 360-alpha
        if (theta1 > (360 - alpha))
        {
            for (auto &it : dist2people)
            {
                if (it.second.second > t3 && t2 + fabs(it.second.second - 360) < 2 * alpha)
                    return true;
            }
        }
    }
    return false;
}
} // namespace Navigators
