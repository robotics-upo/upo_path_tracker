/*
* Navigator Class: For holonomic and no holonomic robots
* Rafael Rey Arcenegui 2019, UPO
*/
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>

namespace Navigators
{

//Displacement Class functions
//
//
Displacement::Displacement(ros::NodeHandle *n, SecurityMargin *margin_, sensor_msgs::LaserScan *laserScan_, tf2_ros::Buffer *tfBuffer_, bool holon_)
{
    nh = n;
    //Right now we will put the ARCO cmd vel topic but in the future it will selectable
    twist_pub = nh->advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);
    muving_state_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/muving_state", 10);
    goal_reached_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_reached", 1);

    //This fields will always be zero for an AGV
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;

    holonomic = holon_;

    //To do: include in parameters configuration (pass it from the node from a launch file)
    angularMaxSpeed = 0.3;
    linearMaxSpeed = 0.2;
    angleMargin = 15;
    distMargin = 0.35;

    //By default we havent arrive anywhere at start
    goalReached.data = false;
    //Pointer to the security margin object createdÃ§
    margin = margin_;
    laserScan = laserScan_;
    tfBuffer = tfBuffer_;
}
/*Parameters to include in the launch file 
    angleMargin
    distMargin
    angularMaxSpeed
    linearMaxSpeed
*/
//Under
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
    ROS_WARN("Yaw dif: %.2f", yawDif);
    if (fabs(yawDif) > angleMargin_)
    {
        Wz = angularMaxSpeed * yawDif / fabs(yawDif);

        if (fabs(yawDif) < angleMargin_ * 2)
            Wz /= 2;
    }
    else if (goal)
    {
        goalReached.data = true;
    }
    if (pub)
    { //If true it published the pure rotation, else it only set Wz values according to speeds and angleMargin gave by the user
        Vx = 0;
        Vy = 0;
        Displacement::publishCmdVel();
        ROS_INFO("Wz: %.2f", Wz);
    }
}
//Function used to aproximate to a near point slowly, like when the robot has arrive
//to a goal(or entered in the dist margin) but it a little  bit far from it
void Displacement::aproximateTo(geometry_msgs::PoseStamped *pose)
{
    geometry_msgs::PoseStamped p;
    //Once we have the pose respect to the base link we can know how we need to move
    p = transformPose(*pose, "map", "base_link");
    //We will try with /3
    Vx = p.pose.position.x / 3;
    Vy = p.pose.position.y / 3;
    Displacement::setRobotOrientation(getYawFromQuat(pose->pose.orientation), 0, 0, 0.08, 3);

    Displacement::publishCmdVel();
}
/*
  *   The idea is to start muving in the direction of the nextPoint relative to base_link frame
  *   But, to give some preference to X direction of muvement(forward) because of reasons(in ARCO for example we have blind areas in the laterals)
  *   So we will give velocity v=(vx,vy) in the direction of the next point, but at the same time the robot will rotate to face the point and thus
  *   the mouvement will finally be in x direction. 
  *   Also, we will use differents top speed for x and y because is more dangerous to move in y direction than x. 
*/
void Displacement::moveHolon(double theta, double dist2GlobalGoal_)
{

    //First we want to know where is nextPoint with respect to the base_link frame

    //PoseStamp nextPoseBlFrame = Displacement::transformPose(*nextPoint, "map", "base_link");
    //double theta = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);

    Vx = cos(theta) * linearMaxSpeed;
    Vy = sin(theta) * linearMaxSpeed * 0.7;

    ROS_WARN("THETA %.2f", theta);
    ROS_WARN("V: [%.2f, %.2f]", Vx, Vy);
    //We have already the Vx and Vy velocities, now we want the rotation to progressively face the next point

    //Theta is in the interval [-180,180]
    //theta/fabs(theta) does the sign job automatically, if theta<0, theta/fabs(theta) = -1 :D
    //if the angle to the next point is too big we do a little bit of rotation in place

    if (fabs(theta) > 110 * M_PI / 180)
    {
        Wz = angularMaxSpeed * theta / fabs(theta);
    }
    else if (fabs(theta) > angleMargin * M_PI / 180)
    {
        Wz = angularMaxSpeed * theta / fabs(theta);
        if (fabs(theta) < 2 * angleMargin * M_PI / 180)
        {
            Wz = (angularMaxSpeed / 2) * theta / fabs(theta);
        }
    }

    Displacement::publishCmdVel();
}

void Displacement::moveNonHolon(double angle2NextPoint_, double dist2GlobalGoal_)
{
    if (fabs(angle2NextPoint_) > angleMargin * M_PI / 180)
    {
        Wz = angularMaxSpeed * angle2NextPoint_ / fabs(angle2NextPoint_);
    }
    else
    {
        Vx = linearMaxSpeed;
        Vy = 0;
        Wz = fabs(angle2NextPoint_) / 3 * angle2NextPoint_ / fabs(angle2NextPoint_);

        if (dist2GlobalGoal_ < distMargin * 2)
            Vx *= 0.4;
    }
}
//Non - holonomic navigation function
void Displacement::navigate(trajectory_msgs::MultiDOFJointTrajectoryPoint *nextPoint, geometry_msgs::PoseStamped *globalGoalMapFrame)
{

    Vx = 0;
    Vy = 0;
    Wz = 0;
    //The trajectory and the next_point transform is received in the map frame so we hace to transform it
    //to base_link frame to work with it
    PoseStamp nextPoseBlFrame;

    nextPoseBlFrame = Displacement::transformPose(*nextPoint, "map", "base_link");

    angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);
    dist2NextPoint = Displacement::euclideanDistance(nextPoseBlFrame);

    globalGoalPose = Displacement::transformPose(*globalGoalMapFrame, "map", "base_link");

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
        Displacement::aproximateTo(globalGoalMapFrame);
    }
    else
    {
        if (holonomic)
        {
            Displacement::moveHolon(angle2NextPoint, dist2GlobalGoal);
        }
        else
        {
            Displacement::moveNonHolon(angle2NextPoint, dist2GlobalGoal);
        }
    }
    /*
    if (dist2GlobalGoal < distMargin && !goalReached.data)
    {
        Displacement::setRobotOrientation(globalGoalMapFrame->pose.orientation, 1, 1, angularMaxSpeed, angleMargin);
    }
    else
    { //Rotation in place
        if (fabs(angle2NextPoint) > angleMargin * M_PI / 180)
        {
            Wz = angularMaxSpeed * angle2NextPoint / fabs(angle2NextPoint);
        }
        else
        {
            Vx = linearMaxSpeed;
            Vy = 0;
            Wz = fabs(angle2NextPoint) / 3 * angle2NextPoint / fabs(angle2NextPoint);

            if (dist2GlobalGoal < distMargin * 2)
                Vx *= 0.4;
        }

        Displacement::publishCmdVel();
    }
    if (goalReached.data)
    {
    }
    */
}
bool Displacement::finished()
{
    return goalReached.data;
}
//When the node gets a global goal from the callback the status variable of the node is set to false and it resets the goalReached.data flag
//from the class
void Displacement::setGoalReachedFlag(bool status_)
{
    goalReached.data = false;
    if (status_)
        goalReached.data = true;
}
void Displacement::publishCmdVel()
{

    muvingState.data = true;

    vel.angular.z = Wz;
    vel.linear.x = Vx;
    vel.linear.y = Vy;

    if (Vx == 0 && Vy == 0 && Wz == 0)
        muvingState.data = false; //It is used to publish to the topic the muving state of the robot

    if (margin->canIMove(laserScan))
    {
        twist_pub.publish(vel);
    }

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
