/*
* Navigator Class: For holonomic and no holonomic robots
* Rafael Rey Arcenegui 2019, UPO
*/
#include <navigator/navigator.hpp>

namespace Navigators
{

SecurityMargin::SecurityMargin(ros::NodeHandle *n)
{
    SecurityMargin::setParams(true, 721, 1.6, 0.2, 0.6, n);
    SecurityMargin::buildArrays();
}
SecurityMargin::SecurityMargin(bool onlyFront_, int laserArrayMsgLen_, float f_, float innerSecDist_, float extSecDist_, ros::NodeHandle *n)
{
    SecurityMargin::setParams(onlyFront_, laserArrayMsgLen_, f_, innerSecDist_, extSecDist_, n);
    SecurityMargin::buildArrays();
}
void SecurityMargin::setParams(bool onlyFront_, int laserArrayMsgLen_, float f_, float innerSecDist_, float extSecDist_, ros::NodeHandle *n)
{
    //Parse parameters
    onlyFront = onlyFront_;
    laserArrayMsgLen = laserArrayMsgLen_;
    f = f_;
    innerSecDist = innerSecDist_;
    extSecDist = extSecDist_;
    paramsConfigured = true;

    nh = n;
    marker_pub = nh->advertise<RVizMarker>("/securityDistMarkers", 1);

    markerInt.header.frame_id = "base_link";
    markerInt.header.stamp = ros::Time();
    markerInt.ns = "debug";
    markerInt.id = 37;
    markerInt.type = RVizMarker::SPHERE_LIST;
    markerInt.action = RVizMarker::ADD;
    markerInt.pose.orientation.w = 1.0;
    markerInt.scale.x = 1.0;
    markerInt.scale.y = 1.0;
    markerInt.color.a = 1.0;
    markerInt.color.r = 0.0;
    markerInt.color.g = 1.0;
    markerInt.color.b = 0.0;

    markerExt.header.frame_id = "base_link";
    markerExt.header.stamp = ros::Time();
    markerExt.ns = "debug";
    markerExt.id = 12;
    markerExt.type = RVizMarker::SPHERE_LIST;
    markerExt.action = RVizMarker::ADD;
    markerExt.pose.orientation.w = 1.0;
    markerExt.scale.x = 1.0;
    markerExt.scale.y = 1.0;
    markerExt.color.a = 1.0;
    markerExt.color.r = 0.0;
    markerExt.color.g = 1.0;
    markerExt.color.b = 0.0;
}
void SecurityMargin::buildArrays()
{
    double x, y;
    geometry_msgs::Point p;
    p.z = 0;
    for (int i = 0; i < laserArrayMsgLen; i++)
    {
        x = innerSecDist * cos(i / 721 * M_PI - M_PI_2);
        y = f * innerSecDist * sin(i / 721 * M_PI - M_PI_2);
        secArray.push_back(sqrtf(x * x + y * y));
        p.x = x;
        p.y = y;
        markerInt.points.push_back(p);

        x *= extSecDist / innerSecDist;
        y *= extSecDist / innerSecDist;
        secArrayExt.push_back(sqrtf(x * x + y * y));
        p.x = x;
        p.y = y;
        markerExt.points.push_back(p);
    }
}

void SecurityMargin::publishRvizMarkers()
{
    //publish r
    marker_pub.publish(markerInt);
    marker_pub.publish(markerExt);
}

//Displacement Class functions
//
//
Displacement::Displacement(ros::NodeHandle *n)
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
}/*
void Displacement::navigate(){

    Vx = 0;
    Vy = 0;
    Wz = 0;

    PoseStamp globalGoalPose;
    //The trajectory and the next_point transform is received in the map frame so we hace to transform it
    //to base_link frame to work with it
    PoseStamp next_pointpose, next_pose;

    next_pointpose.pose.position.x = next_point.transform.translation.x;
    next_pointpose.pose.position.y = next_point.transform.translation.y;
    next_pointpose.pose.orientation = next_point.transform.rotation;

    next_pose = transformPose(next_pointpose, "map", "base_link");
    double angle2Nextpoint = atan2(next_pose.pose.position.y, next_pose.pose.position.x);

    while (angle2Nextpoint > 2 * M_PI)
    {
        angle2Nextpoint -= 2 * M_PI;
    }
    if(angle2Nextpoint > M_PI ){
        angle2Nextpoint-=2*M_PI;
    }
    if(angle2Nextpoint < -M_PI){
        angle2Nextpoint += 2*M_PI;
    }
    double dist2NextPoint = Displacement::euclideanDistance(next_pose);

    globalGoalPose = transformPose(globalGoal, "map", "base_link");
    dist2GlobalPoint = euclideanDistance(globalGoalPose);
    double angle2GlobalGoal = Displacement::getYawFromQuat(globalGoalPose.pose.orientation);

    if (dist2GlobalPoint < distMargin && !goalReached.data)
    {
        
        angle2Nextpoint = atan2(globalGoalPose.pose.position.y, globalGoalPose.pose.position.x);
        while (fabs(angle2GlobalGoal) > angleMargin)
        {
            globalGoalPose = transformPose(globalGoal, "map", "base_link");
            angle2GlobalGoal = Displacement::getYawFromQuat(globalGoalPose.pose.orientation);
            Wz = angularMaxSpeed;

            if (angle2GlobalGoal < 0)
                Wz *= -1;
            if (fabs(angle2Nextpoint) < angleMargin * 2)
                Wz /= 2;
           
            Displacement::publishCmdVel(0, 0,Wz);
        }
        finalOrientationOk = true;
        goalReached.data = true;
        Displacement::publishCmdVel();
    }
    else if (!checkObstacles(&secArray, 1) && !goalReached.data)
    { //Rotation in place
        if (fabs(angle2Nextpoint) > (angleMargin+5) * M_PI / 180)
        {
            Wz = angularMaxSpeed;
            if (angle2Nextpoint < 0)
                Wz *= -1;
           
             Displacement::publishCmdVel();
        }
        else
        {
            Vx = 0.4;
            Wz = fabs(angle2Nextpoint) * 13 / (5 * M_PI);

            if (angle2Nextpoint < 0)
                Wz *= -1;

            if (dist2GlobalPoint < distMargin * 2)
                Vx *= 0.4;
             Displacement::publishCmdVel();
        }
    }
    else if (checkObstacles(&secArray, 1))
    {
         Displacement::publishCmdVel();
    }
}*/
void Displacement::publishCmdVel()
{
    muvingState.data = true;
    vel.angular.z = Wz;
    vel.linear.x = Vx;
    vel.linear.y = Vy;

    if (Vx == 0 && Vy == 0 && Wz == 0)
        muvingState.data = false;

    twist_pub.publish(vel);
    muving_state_pub.publish(muvingState);
    goal_reached_pub.publish(goalReached);
}

PoseStamp Displacement::transformPose(PoseStamp originalPose, std::string from, std::string to, tf2_ros::Buffer *tfBuffer)
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
