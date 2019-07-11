/*
* Navigator Class: For holonomic and no holonomic robots
* Rafael Rey Arcenegui 2019, UPO
*/
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>

namespace Navigators
{

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
    //ros::param::get("/nav_node/traj_timeout", traj_timeout);
    ros::param::get("/nav_node/start_orientate_dist", startOrientateDist);

    if (delta > traj_timeout)
    {
        if(!outOfTime){
            outOfTime = true;
            publishZeroVelocity();
        }        
    }else{
        outOfTime = false;
    }
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
//Displacement Class functions
Displacement::Displacement(ros::NodeHandle *n, SecurityMargin *margin_, tf2_ros::Buffer *tfBuffer_)
{
    nh = n;
    //Pointer to the security margin object createdÃ§
    margin = margin_;
    tfBuffer = tfBuffer_;

    //Right now we will put the ARCO cmd vel topic but in the future it will selectable
    twist_pub = nh->advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);
    muving_state_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/muving_state", 1);
    goal_reached_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_reached", 1);
    
    aproach_maneouvre_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/aproach_manoeuvre",1);

    dist2goal_pub = nh->advertise<std_msgs::Float32>("/dist2goal",0);
    dist2goal.data = 0;
    
    nh->param("/nav_node/do_navigate", do_navigate, (bool)true);
    nh->param("/nav_node/holonomic", holonomic, (bool)true);
    nh->param("/nav_node/angular_max_speed", angularMaxSpeed, (float)0.5);
    nh->param("/nav_node/linear_max_speed", linearMaxSpeed, (float)0.3);
    nh->param("/nav_node/angle_margin", angleMargin, (float)10);
    nh->param("/nav_node/dist_margin", distMargin, (float)0.35);
    nh->param("/nav_node/a", a, (float)5);
    nh->param("/nav_node/b", b, (float)5);
    nh->param("/nav_node/traj_timeout", traj_timeout, (float)0.025);
    nh->param("/nav_node/start_orientate_dist", startOrientateDist, (float)0.025);
    nh->param("/nav_node/robot_base_frame", robot_frame, (string) "base_link");
    nh->param("/nav_node/world_frame", world_frame, (string) "map");
    nh->param("/nav_node/aproach_factory", factoryAproach, (bool)0);
    nh->param("/nav_node/aproach_distance",aproachDistance, (float)1);
    old_b = b;

    //Start flags values
    //Flags to publish
    possible_to_move.data = true;
    localGoalOcc.data = false;
    goalReached.data = false;
    outOfTime = false;

    //Flags for internal states
    homePublished = false;
    trajReceived = false;
    aproxComplete = false;

    tr0_catch = false;
    n_goals = 0;    
    alpha = 30;
    traj_n_received=false;
    manoeuvreStatus.data=false;
}
void Displacement::occLocalGoalCb(const std_msgs::Bool::ConstPtr &msg)
{
    localGoalOcc = *msg;
    if (localGoalOcc.data)
    {
        publishZeroVelocity();
    }
}
void Displacement::impossibleMoveCb(const std_msgs::Bool::ConstPtr &msg)
{
    possible_to_move.data = !(msg->data);
}
void Displacement::trackedPersonCb(const people_msgs::People::ConstPtr &pl)
{
    peopl = *pl;
    computeDistanceToPeople();
    printPeople();
}
void Displacement::trajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
    nextPoint = trj->points[trj->points.size() > 1 ? 1 : 0];
    trajReceived = true;
    delta = ros::Time::now().toSec() - trj->header.stamp.toSec();
}

//Used to calculate distance from base_link to global goal
void Displacement::globalGoalCb(const geometry_msgs::PoseStampedConstPtr &globGoal_)
{    
    globalGoal = *globGoal_;
    setGoalReachedFlag(0);
    ros::param::get("/nav_node/traj_timeout", traj_timeout);
    trajReceived = false;
    aproxComplete = false;
    tr0_catch = false;
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
        setGoalReachedFlag(1);
        traj_timeout = std::numeric_limits<float>::max();
    }

    if (pub)
        publishCmdVel();
}
void Displacement::aproximateTo(geometry_msgs::PoseStamped *pose, bool isGoal, bool isHome)
{

    geometry_msgs::PoseStamped p = transformPose(*pose, world_frame, robot_frame);

    Vx = p.pose.position.x/2;
    Vy = p.pose.position.y/2;

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

    
    if (fabs(angle2NextPoint) > d2rad(angleMargin))
    {
        Wz = getVel(angularMaxSpeed, 4*a, angle2NextPoint);
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
    }

    if (dist2GlobalGoal < startOrientateDist)
        setRobotOrientation(finalYaw, 0, 0, angularMaxSpeed, 10);
}
void Displacement::moveNonHolon()
{
    if (fabs(angle2NextPoint) > d2rad(angleMargin))
    {
        Wz = getVel(angularMaxSpeed, a, angle2NextPoint);
    }
    else
    {
        Vx = getVel(linearMaxSpeed, b, dist2GlobalGoal);
        Vy = 0;
        Wz = getVel(angularMaxSpeed, a / 2, angle2NextPoint);
    }
}

void Displacement::navigate()
{
    refreshParams();

    if (trajReceived && !goalReached.data && do_navigate && !localGoalOcc.data && possible_to_move.data)
    {
        if (aproxComplete)
        {
            aproxComplete = false;
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

        if (dist2GlobalGoal < distMargin && !goalReached.data)
        {
            ROS_INFO_ONCE("Maniobra de aproximacion");
            aproximateTo(&globalGoal, 1, 0);
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
    if (goalReached.data && !aproxComplete && factoryAproach)
    {
        
        if (!tr0_catch)
        {
            dlt.x =0;
            dlt.y = 0;
            tr1.transform.translation.x = 0;
            tr1.transform.translation.y = 0;
            try
            {
                tr0 = tfBuffer->lookupTransform(world_frame, robot_frame, ros::Time(0));
                tr0_catch = true;
                manoeuvreStatus.data = true;
                aproach_maneouvre_pub.publish(manoeuvreStatus);
                
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("No transform %s", ex.what());
            }

            vel.linear.y = 0;
            vel.angular.z = 0;
        }
        else
        {
            try
            {
                tr1 = tfBuffer->lookupTransform(world_frame, robot_frame, ros::Time(0));
                dlt.x = tr1.transform.translation.x - tr0.transform.translation.x;
                dlt.y = tr1.transform.translation.y - tr0.transform.translation.y;
                vel.linear.x = -0.05-1*getVel(0.2, 2, 1 - sqrtf(dlt.x * dlt.x + dlt.y * dlt.y));
                if (sqrtf(dlt.x * dlt.x + dlt.y * dlt.y) < aproachDistance)
                {
                    twist_pub.publish(vel);
                }
                else
                {
                    aproxComplete = true;
                    tr0_catch = false;
                    publishZeroVelocity();
                    manoeuvreStatus.data = false;
                    aproach_maneouvre_pub.publish(manoeuvreStatus);
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("No transform %s", ex.what());
            }
            
                        
        }
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

    muvingState.data = false;

    twist_pub.publish(vel);
    muving_state_pub.publish(muvingState);
}
void Displacement::setGoalReachedFlag(bool status_)
{
    if (status_)
    {
        goalReached.data = true;
        publishZeroVelocity();   
    }else{
        goalReached.data = false;
    }
    goal_reached_pub.publish(goalReached);
}
void Displacement::publishCmdVel()
{

    muvingState.data = true;

    if (Vx == 0 && Vy == 0 && Wz == 0)
        muvingState.data = false;

    if (margin->canIMove())
    {
        vel.angular.z = Wz;
        vel.linear.x = Vx;
        vel.linear.y = Vy;
        twist_pub.publish(vel);
        muving_state_pub.publish(muvingState);
    }
    else
    {
        publishZeroVelocity();
    }
   
}

PoseStamp Displacement::transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to)
{
    PoseStamp pose;
    pose.header.frame_id = from;
    pose.header.seq = rand();
    pose.header.stamp = ros::Time(0);

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
