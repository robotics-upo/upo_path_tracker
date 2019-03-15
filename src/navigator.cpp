/*
* Navigator Class: For holonomic and no holonomic robots
* Rafael Rey Arcenegui 2019, UPO
*/
#include <navigator/navigator.hpp>

namespace Navigators
{

SecurityMargin::SecurityMargin(ros::NodeHandle *n)
{
    SecurityMargin::setParams(true, 721, 1.6, 0.2, 0.6, 15, n);
    SecurityMargin::buildArrays();
}
SecurityMargin::SecurityMargin(bool onlyFront_, int laserArrayMsgLen_, float f_, float innerSecDist_, float extSecDist_, int laserSecurityAngle_, ros::NodeHandle *n)
{
    SecurityMargin::setParams(onlyFront_, laserArrayMsgLen_, f_, innerSecDist_, extSecDist_, laserSecurityAngle_, n);
    SecurityMargin::buildArrays();
}
void SecurityMargin::setParams(bool onlyFront_, int laserArrayMsgLen_, float f_, float innerSecDist_, float extSecDist_, int laserSecurityAngle_, ros::NodeHandle *n)
{
    //Parse parameters
    onlyFront = onlyFront_;
    laserArrayMsgLen = laserArrayMsgLen_;
    f = f_;
    innerSecDist = innerSecDist_;
    extSecDist = extSecDist_;
    paramsConfigured = true;

    laserSecurityAngle = ceil(laserSecurityAngle_ * (laserArrayMsgLen / 180));
    nh = n;
    marker_pub = nh->advertise<RVizMarker>("/securityDistMarkers", 1);

    markerInt.header.frame_id = "front_laser_link";
    markerInt.header.stamp = ros::Time();
    markerInt.ns = "debug";
    markerInt.id = 37;
    markerInt.type = RVizMarker::SPHERE_LIST;
    markerInt.action = RVizMarker::ADD;
    markerInt.pose.orientation.w = 1.0;
    markerInt.scale.x = 0.1;
    markerInt.scale.y = 0.1;
    markerInt.color.a = 1.0;
    markerInt.color.r = 1.0;
    markerInt.color.g = 0.0;
    markerInt.color.b = 0.0;

    markerExt.header.frame_id = "front_laser_link";
    markerExt.header.stamp = ros::Time();
    markerExt.ns = "debug";
    markerExt.id = 12;
    markerExt.type = RVizMarker::SPHERE_LIST;
    markerExt.action = RVizMarker::ADD;
    markerExt.pose.orientation.w = 1.0;
    markerExt.scale.x = 0.1;
    markerExt.scale.y = 0.1;
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
    markerInt.points.clear();
    markerExt.points.clear();
    for (double i = 0; i < laserArrayMsgLen; i++)
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
   
    marker_pub.publish(markerInt);
    marker_pub.publish(markerExt);
}
//whichOne: boolean to select which ellipse to evaluate, 1 for ext and 0 for inner
bool SecurityMargin::checkObstacles(bool whichOne, sensor_msgs::LaserScan *scan)
{
    for (int i = laserSecurityAngle; i < laserArrayMsgLen - laserSecurityAngle; i++)
    {
        //TODO: Change 0.02 to a parameter or filter the laser information
        //It compared to secArray or secArrayExt depending the value of whichOne
        if (scan->ranges.at(i) < (whichOne ? secArrayExt.at(i) : secArray.at(i) ) && scan->ranges.at(i) > scan->range_min)
        {
            if (!whichOne)
            {
                isInsideDangerousArea = true;
                securityAreaOccup = true;
                
            }
            return true;
        }
    }
    //If we ara evaluating the exterior ellipses, and it didn't find nothing inside this ellipse, it changes the flags values
    if (whichOne)
    {
        isInsideDangerousArea = false;
        securityAreaOccup = false;
    }
    return false;
}
bool SecurityMargin::dangerAreaFree()
{
    return isInsideDangerousArea;
}

bool SecurityMargin::securityAreaFree()
{
    return !securityAreaOccup;
}

//Displacement Class functions
//
//
Displacement::Displacement(ros::NodeHandle *n, Navigators::SecurityMargin *margin_, sensor_msgs::LaserScan *laserScan_, tf2_ros::Buffer *tfBuffer_)
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
    //To do: include in parameters configuration
    angularMaxSpeed = 0.5;
    linearMaxSpeed = 0.4;
    angleMargin = 15;
    distMargin = 0.35;
    //By default we havent arrive anywhere at start
    goalReached.data=false;
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
//Under construction
void Displacement::setRobotOrientation(float yaw, bool goal){
    geometry_msgs::Quaternion q;
    tf2::Quaternion q2;
    q2.setRPY(0,0,yaw);
    q = tf2::toMsg(q2);
    Displacement::setRobotOrientation(q,goal);
}
void Displacement::setRobotOrientation(geometry_msgs::Quaternion q, bool goal){
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

    robotPose = Displacement::transformPose(robotPose,"base_link","map");
    float robotYaw = Displacement::getYawFromQuat(robotPose.pose.orientation);
    float finalYaw = Displacement::getYawFromQuat(q);

    if(robotYaw < 0)
        robotYaw+=360;
    if(finalYaw < 0)
        finalYaw+=360;

    float yawDif = robotYaw-finalYaw;

    if( fabs(yawDif) > angleMargin){
        Wz = angularMaxSpeed;

        if( yawDif > 0 )
            Wz*=-1;
        if(fabs(yawDif) < angleMargin*2)
            Wz/=2;
    
    }else if(goal){
        goalReached.data = true;
    }

    Displacement::publishCmdVel();
}
void Displacement::navigate(trajectory_msgs::MultiDOFJointTrajectoryPoint *nextPoint, geometry_msgs::PoseStamped *globalGoalMapFrame)
{

    Vx = 0;
    Vy = 0;
    Wz = 0;
    //0 for positive, 1 for negative angle
    bool angleSign = 0;
    //The trajectory and the next_point transform is received in the map frame so we hace to transform it
    //to base_link frame to work with it
    PoseStamp nextPoseMapFrame, nextPoseBlFrame;

    nextPoseMapFrame.pose.position.x = nextPoint->transforms[0].translation.x;
    nextPoseMapFrame.pose.position.y = nextPoint->transforms[0].translation.y;
    nextPoseMapFrame.pose.orientation = nextPoint->transforms[0].rotation;

    nextPoseBlFrame = Displacement::transformPose(nextPoseMapFrame, "map", "base_link");
    
    angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);
    if(angle2NextPoint < 0){
        angleSign=1;
        angle2NextPoint*=-1;
    }
    dist2NextPoint = Displacement::euclideanDistance(nextPoseBlFrame);

    globalGoalPose = Displacement::transformPose(*globalGoalMapFrame, "map", "base_link");
    
    dist2GlobalGoal = Displacement::euclideanDistance(globalGoalPose);
    angle2GlobalGoal = Displacement::getYawFromQuat(globalGoalPose.pose.orientation);
    

    if (dist2GlobalGoal < distMargin && !goalReached.data)
    {
        Displacement::setRobotOrientation(globalGoalMapFrame->pose.orientation,1);
    }
    else if (!margin->checkObstacles(0, laserScan) && !goalReached.data)
    { //Rotation in place
        
        if (angle2NextPoint > (angleMargin + 5) * M_PI / 180)
        {
            ROS_WARN("rotatin in place");
            Wz = angularMaxSpeed;
            if (angleSign)
                Wz *= -1;
            if(angle2NextPoint < angleMargin*2)
                Wz/=2;
           
        }
        else
        {
            ROS_WARN("going forwaaa");
            Vx = 0.4;
            Wz = angle2NextPoint * 13 / (5 * M_PI);

            if (angleSign)
                Wz *= -1;

            if (dist2GlobalGoal < distMargin * 2)
                Vx *= 0.4;
        }

        Displacement::publishCmdVel();
    }
    else if (margin->checkObstacles(0, laserScan))
    {
        Displacement::publishCmdVel();   
    }
}
bool Displacement::finished()
{
    return goalReached.data;
}
void Displacement::setGoalReachedFlag(bool status_){
    goalReached.data = false;
    if(status_)
        goalReached.data = true;
       
}
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
