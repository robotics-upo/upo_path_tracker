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
Displacement::Displacement(ros::NodeHandle *n, SecurityMargin *margin_, sensor_msgs::LaserScan *laserScan_, tf2_ros::Buffer *tfBuffer_)
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

    //To do: include in parameters configuration (pass it from the node from a launch file)
    angularMaxSpeed = 0.5;
    linearMaxSpeed = 0.4;
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

    if (fabs(yawDif) > angleMargin_)
    {
        Wz = angularMaxSpeed*yawDif/fabs(yawDif);

        if (fabs(yawDif) < angleMargin_ * 2)
            Wz /= 2;
    }
    else if (goal)
    {
        goalReached.data = true;
    }
    if (pub){//If true it published the pure rotation, else it only set Wz values according to speeds and angleMargin gave by the user 
        Vx=0;
        Vy=0;
        Displacement::publishCmdVel();
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
    Displacement::setRobotOrientation(getYawFromQuat(pose->pose.orientation),0,0,0.08,3);

    if (margin->checkObstacles(0, laserScan))
        Displacement::publishCmdVel();
    
}
/*
  *   The idea is to start muving in the direction of the nextPoint relative to base_link frame
  *   But, to give some preference to X direction of muvement(forward) because of reasons(in ARCO for example we have blind areas in the laterals)
  *   So we will give velocity v=(vx,vy) in the direction of the next point, but at the same time the robot will rotate to face the point and thus
  *   the mouvement will finally be in x direction. 
  *   Also, we will use differents top speed for x and y because is more dangerous to move in y direction than x. 
*/
void Displacement::navigate_h(trajectory_msgs::MultiDOFJointTrajectoryPoint *nextPoint, geometry_msgs::PoseStamped *globalGoalMapFrame){

   
   //First we want to know where is nextPoint with respect to the base_link frame

    float maxLinearXSpeed = linearMaxSpeed;
    float maxLinearYSpeed = linearMaxSpeed*0.7;

    PoseStamp nextPoseBlFrame = Displacement::transformPose(*nextPoint, "map", "base_link");
    
    float mod = sqrtf(pow(nextPoseBlFrame.pose.position.x,2)+pow(nextPoseBlFrame.pose.position.y,2));
    geometry_msgs::Vector3 dirNextPoint;

    //Unitary direction module
    dirNextPoint.x = nextPoseBlFrame.pose.position.x/mod; 
    dirNextPoint.y = nextPoseBlFrame.pose.position.y/mod; 

    Vx = dirNextPoint.x * maxLinearXSpeed;
    Vx = dirNextPoint.y * maxLinearYSpeed;

    //We have already the Vx and Vy velocities, now we want the rotation to progressively face the next point 
    float theta = atan2(dirNextPoint.y,dirNextPoint.x)*180/M_PI;
    //Theta is in the interval [-180,180] 
    //theta/fabs(theta) does the sign job automatically, if theta<0, theta/fabs(theta) = -1 :D
    if(fabs(theta)>angleMargin*2){
        Wz = angularMaxSpeed*theta/fabs(theta);
    }else{
        Wz = angularMaxSpeed/2*theta/fabs(theta);
    }

    if(margin->canIMove(laserScan))
        Displacement::publishCmdVel();


}
//Non - holonomic navigation function
void Displacement::navigate_nh(trajectory_msgs::MultiDOFJointTrajectoryPoint *nextPoint, geometry_msgs::PoseStamped *globalGoalMapFrame)
{

    Vx = 0;
    Vy = 0;
    Wz = 0;
    //The trajectory and the next_point transform is received in the map frame so we hace to transform it
    //to base_link frame to work with it
    PoseStamp nextPoseBlFrame;

    nextPoseBlFrame = Displacement::transformPose(*nextPoint, "map", "base_link");

    angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x) / M_PI * 180;
    dist2NextPoint = Displacement::euclideanDistance(nextPoseBlFrame);

    globalGoalPose = Displacement::transformPose(*globalGoalMapFrame, "map", "base_link");

    dist2GlobalGoal = Displacement::euclideanDistance(globalGoalPose);
    angle2GlobalGoal = Displacement::getYawFromQuat(globalGoalPose.pose.orientation);

    if (dist2GlobalGoal < distMargin && !goalReached.data)
    {
        Displacement::setRobotOrientation(globalGoalMapFrame->pose.orientation, 1, 1,angularMaxSpeed, angleMargin);
    }
    else if (!margin->checkObstacles(0, laserScan) && !goalReached.data)
    { //Rotation in place

        if (angle2NextPoint > (angleMargin + 5))
        {
            Displacement::setRobotOrientation(angle2NextPoint, 0, 1,angularMaxSpeed, angleMargin);
        }
        else
        {
            Vx = 0.4;
            Wz = angle2NextPoint * 13 / (5 * M_PI);

            if (angle2NextPoint < 0)
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
