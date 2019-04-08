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
    ros::param::get("/nav_node/linear_max_speed_x", linearMaxSpeedX);
    ros::param::get("/nav_node/linear_max_speed_y", linearMaxSpeedY);
    ros::param::get("/nav_node/angle_margin", angleMargin);
    ros::param::get("/nav_node/dist_margin", distMargin);
}
//Displacement Class functions
Displacement::Displacement(ros::NodeHandle *n, SecurityMargin *margin_, tf2_ros::Buffer *tfBuffer_)
{
    nh = n;

    //Right now we will put the ARCO cmd vel topic but in the future it will selectable
    twist_pub = nh->advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);
    muving_state_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/muving_state", 10);
    goal_reached_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_reached", 1);
    goal_pub = nh->advertise<PoseStamp>("/move_base_simple/goal", 1); //Used for homing
    leds_pub = nh->advertise<std_msgs::UInt8MultiArray>("/idmind_sensors/set_leds", 10);

    nh->param("/nav_node/do_navigate", do_navigate, (bool)true);
    nh->param("/nav_node/holonomic", holonomic, (bool)true);
    nh->param("/nav_node/angular_max_speed", angularMaxSpeed, (float)0.5);
    nh->param("/nav_node/linear_max_speed_x", linearMaxSpeedX, (float)0.3);
    nh->param("/nav_node/linear_max_speed_y", linearMaxSpeedY, (float)0.2);
    nh->param("/nav_node/angle_margin", angleMargin, (float)15);
    nh->param("/nav_node/dist_margin", distMargin, (float)0.35);

    //Pointer to the security margin object createdÃ§
    margin = margin_;

    tfBuffer = tfBuffer_;

    //These fields will always be zero for an AGV
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;

    //By default we havent arrive anywhere at start
    goalReached.data = false;
    homePublished = false;
    trajReceived = false;
}
void Displacement::trajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
    trajReceived = true;
    goalReached.data = false;
    int i = 0;

    if (trj->points.size() > 1)
        i = 1;

    nextPoint = trj->points[i];
}
//Used to calculate distance from base_link to global goal
void Displacement::globalGoalCb(const geometry_msgs::PoseStampedConstPtr &globGoal_)
{
    goalReached.data = false;
    globalGoal = *globGoal_;
}
void Displacement::setRobotOrientation(geometry_msgs::Quaternion q, bool goal, bool pub, float speed, float angleMargin_)
{
    Displacement::setRobotOrientation(Displacement::getYawFromQuat(q), goal, pub, speed, angleMargin_);
}
void Displacement::setRobotOrientation(float finalYaw, bool goal, bool pub, float speed, float angleMargin_)
{
    //We receive the quaternion q in the frame map. We have to know the orientation of the robot
    //also in the map frame

    PoseStamp robotPose;
    robotPose.header.frame_id = "base_link";
    robotPose.header.stamp = ros::Time(0);
    robotPose.header.seq = rand();

    robotPose.pose.position.x = 0;
    robotPose.pose.position.y = 0;
    robotPose.pose.position.z = 0;

    robotPose.pose.orientation.w = 1;

    robotPose = Displacement::transformPose(robotPose, "base_link", "map");
    float robotYaw = Displacement::getYawFromQuat(robotPose.pose.orientation);

    if (robotYaw < 0)
        robotYaw += 360;
    if (finalYaw < 0)
        finalYaw += 360;

    float yawDif = finalYaw - robotYaw;

    if (fabs(yawDif) > 180)
    {
        if (yawDif < 0)
        {
            yawDif += 360;
        }
        else
        {
            yawDif -= 360;
        }
    }
    if (fabs(yawDif) > angleMargin_)
    {
        Wz = 2 * (yawDif * (speed - 0.1) / (180 - angleMargin_) + speed - 180 / (180 - angleMargin_) * (speed - 0.1));
    }
    else if (goal)
    {
        Displacement::setGoalReachedFlag(1);
    }

    if (pub)
    {

        Displacement::publishCmdVel();
    }
}
//Function used to aproximate to a near point slowly, like when the robot has arrive
//to a goal(or entered in the dist margin) but it a little  bit far from it
void Displacement::goHomeLab()
{
    if (!homePublished) //HomePublished flag is used to publish home position only once
    {
        PoseStamp home;
        home.header.frame_id = "map";
        home.header.seq = rand();
        home.header.stamp = ros::Time(0);

        home.pose.position.x = 4.1;
        home.pose.position.y = 3.15;
        home.pose.position.z = 0;

        home.pose.orientation.w = 0;
        home.pose.orientation.z = -1;

        goal_pub.publish(home);
        homePublished = true;
    }
}
void Displacement::aproximateTo(geometry_msgs::PoseStamped *pose, bool isGoal, bool isHome)
{
    if (!isHome)
    {
        geometry_msgs::PoseStamped p;

        p = transformPose(*pose, "map", "base_link");

        Vx = p.pose.position.x;
        Vy = p.pose.position.y;

        Displacement::setRobotOrientation(getYawFromQuat(pose->pose.orientation), isGoal, 0, angularMaxSpeed, 3);

        Displacement::publishCmdVel();
    }
    else if (goalReached.data) //TODO: Make the homing work xD
    {
        Vx = -0.05;
        Displacement::publishCmdVel();
    }
}
/*
  *   The idea is to start muving in the direction of the nextPoint relative to base_link frame
  *   But, to give some preference to X direction of muvement(forward) because of reasons(in ARCO for example we have blind areas in the laterals)
  *   So we will give velocity v=(vx,vy) in the direction of the next point, but at the same time the robot will rotate to face the point and thus
  *   the mouvement will finally be in x direction. 
  *   Also, we will use differents top speed for x and y because is more dangerous to move in y direction than x. 
*/
void Displacement::moveHolon(double theta, double dist2GlobalGoal_, double finalYaw)
{
    Vx = cos(theta) * linearMaxSpeedX;
    Vy = sin(theta) * linearMaxSpeedY;
    if (dist2GlobalGoal_ < distMargin * 1.5)
    {
        Vx /= 2;
        Vy /= 2;
    }
    if (fabs(theta) > M_PI_2 - M_PI / 9 && fabs(theta) < M_PI_2 + M_PI / 9)
    {
    }
    /*if (fabs(theta) > 110 * M_PI / 180)
    {
        Wz = angularMaxSpeed * theta / fabs(theta);
    }*/
    else if (fabs(theta) > angleMargin * M_PI / 180)
    {
        Wz = angularMaxSpeed * theta / fabs(theta);

        if (fabs(theta) < 2 * angleMargin * M_PI / 180)
        {
            Wz = (angularMaxSpeed / 3) * theta / fabs(theta);
        }
    }
    //If we are close to the goal, let's start orientate the robot towards the final yaw
    if (dist2GlobalGoal_ < 1.5)
    {
        Displacement::setRobotOrientation(finalYaw, 0, 0, angularMaxSpeed, 10);
    }

    Displacement::publishCmdVel();
}

void Displacement::moveNonHolon(double angle2NextPoint_, double dist2GlobalGoal_)
{
    if (fabs(angle2NextPoint_) > angleMargin * M_PI / 180)
    {
        Wz = 4 * angle2NextPoint_ * 180 / M_PI * (angularMaxSpeed - 0.15) / (180 - angleMargin) + angularMaxSpeed - 180 / (180 - angleMargin) * (angularMaxSpeed - 0.15);
    }
    else
    {
        Vx = linearMaxSpeedX;
        Vy = 0;
        Wz = fabs(angle2NextPoint_) / 3 * angle2NextPoint_ / fabs(angle2NextPoint_);

        if (dist2GlobalGoal_ < distMargin * 2)
            Vx *= 0.4;
    }
    Displacement::publishCmdVel();
}

void Displacement::navigate(bool isHome)
{
    Displacement::refreshParams();
    if (trajReceived && !goalReached.data && do_navigate)
    {
        Vx = 0;
        Vy = 0;
        Wz = 0;

        PoseStamp nextPoseBlFrame;

        nextPoseBlFrame = Displacement::transformPose(nextPoint, "map", "base_link");

        angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);
        dist2NextPoint = Displacement::euclideanDistance(nextPoseBlFrame);

        globalGoalPose = Displacement::transformPose(globalGoal, "map", "base_link");

        dist2GlobalGoal = Displacement::euclideanDistance(globalGoalPose);
        angle2GlobalGoal = Displacement::getYawFromQuat(globalGoalPose.pose.orientation);

        /*
        ROS_INFO("hEY");
        ROS_INFO("Next point bl frame: [%.2f, %.2f]", nextPoseBlFrame.pose.position.x, nextPoseBlFrame.pose.position.y);
        ROS_INFO("ANGLE 2 next point: %.2f", angle2NextPoint);
        ROS_INFO("Global goal pose bl frame: [%.2f, %.2f]", globalGoalPose.pose.position.x, globalGoalPose.pose.position.y);
        */
        if (dist2GlobalGoal < distMargin && !goalReached.data)
        {
            ROS_INFO_ONCE("Maniobra de aproximacion");
            Displacement::aproximateTo(&globalGoal, 1, isHome);
        }
        else
        {
            if (holonomic)
            {
                Displacement::moveHolon(angle2NextPoint, dist2GlobalGoal, getYawFromQuat(globalGoal.pose.orientation));
            }
            else
            {
                Displacement::moveNonHolon(angle2NextPoint, dist2GlobalGoal);
            }
        }
    }
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
    goal_reached_pub.publish(goalReached);
}
void Displacement::setGoalReachedFlag(bool status_)
{
    goalReached.data = false;
    if (status_)
    {
        goalReached.data = true;
        Displacement::publishZeroVelocity();
    }
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
    }
    else
    {
        vel.angular.z = 0;
        vel.linear.x = 0;
        vel.linear.y = 0;
    }
    twist_pub.publish(vel);
    muving_state_pub.publish(muvingState);
    goal_reached_pub.publish(goalReached);
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
    return Displacement::transformPose(pose, from, to);
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
} // namespace Navigators
