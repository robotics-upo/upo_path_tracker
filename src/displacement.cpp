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
    ros::param::get("/nav_node/traj_timeout", traj_timeout);
    ros::param::get("/nav_node/start_orientate_dist", startOrientateDist);

    if(delta > traj_timeout){
        trajReceived = false;
        //ROS_WARN("CHANGING FLAG");
        Displacement::publishZeroVelocity();
    }
    //The idea is to save the smallest distance to anyone and change the value of b according to this distance
    
    closest.second.first =  std::numeric_limits<float>::max();

    for(auto it = dist2people.begin(); it != dist2people.end(); ++it)
        if(it->second.first < closest.second.first)
            closest.second = it->second;
     
    
    //Now we set 1.5 has a distance to take into account persons
    //TODO: Tener en cuenta solo las personas que entran dentro de un cono apuntando en la dirección de movimiento

    if(closest.second.first < 1.5){
        b = old_b*closest.second.first/1.5;
        //ROS_WARN("B: %.2f",b);
    }
    //ros::param::set("/nav_node/b", b);
    //printf("\n The closest person is %s, %.2f m away from arco", closest.first.c_str(), closest.second);

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
    muving_state_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/muving_state", 10);
    goal_reached_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_reached", 1);
    goal_pub = nh->advertise<PoseStamp>("/move_base_simple/goal", 1); //Used for homing
    leds_pub = nh->advertise<std_msgs::UInt8MultiArray>("/idmind_sensors/set_leds", 10);

    nh->param("/nav_node/do_navigate", do_navigate, (bool)true);
    nh->param("/nav_node/holonomic", holonomic, (bool)true);
    nh->param("/nav_node/angular_max_speed", angularMaxSpeed, (float)0.5);
    nh->param("/nav_node/linear_max_speed", linearMaxSpeed, (float)0.3);
    nh->param("/nav_node/angle_margin", angleMargin, (float)15);
    nh->param("/nav_node/dist_margin", distMargin, (float)0.35);
    nh->param("/nav_node/a", a, (float)5);
    nh->param("/nav_node/b", b, (float)5);
    nh->param("/nav_node/traj_timeout",traj_timeout , (float)0.025);
    nh->param("/nav_node/start_orientate_dist",startOrientateDist , (float)0.025);
    old_b = b;
    possible_to_move.data = true;
    if(!holonomic){

        angleMargin = 10;
        traj_timeout = 0.275;
        b = 2;
        a = 2;
        linearMaxSpeed = 1;
    
    }else{
        startOrientateDist = 1;
        b = 3;
        linearMaxSpeed = 1;
    }
    alpha = 30;
    //By default we havent arrive anywhere at start
    goalReached.data = false;
    homePublished = false;
    trajReceived = false;

}
void Displacement::impossibleMoveCb(const std_msgs::Bool::ConstPtr &msg){
    possible_to_move = *msg;
    possible_to_move.data = !possible_to_move.data;
}
void Displacement::trackedPersonCb(const people_msgs::People::ConstPtr &pl){
    peopl = *pl;
    Displacement::computeDistanceToPeople();
    //Displacement::printPeople();
}
void Displacement::trajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
    trajReceived = true;
    //goalReached.data = false;
    
    delta = ros::Time::now().toSec()- trj->header.stamp.toSec();

    //ROS_INFO("DELTA: %.3f\n now: %.3f",delta,ros::Time::now().toSec());

    nextPoint = trj->points[trj->points.size()>1?1:0];
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
        yawDif-=360*yawDif/fabs(yawDif);
    

    if (fabs(yawDif) > angleMargin_)
    {
        //This one works VERY WELL DONT TOUCH IT 
        Wz = 2 * (yawDif * (speed - 0.1) / (180 - angleMargin_) + speed - 180 / (180 - angleMargin_) * (speed - 0.1));
        
        //Wz = Displacement::getVel(angularMaxSpeed,a,yawDif,2);
    }
    else if (goal)
    {
        Displacement::setGoalReachedFlag(1);
        //ROS_WARN("LLegamos baby");
    }

    if (pub)
        Displacement::publishCmdVel();
    
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
    //if (!isHome)
    //{
    geometry_msgs::PoseStamped p = transformPose(*pose, "map", "base_link");
    
    Vx = p.pose.position.x;
    Vy = p.pose.position.y;
    
    Displacement::setRobotOrientation(getYawFromQuat(pose->pose.orientation), isGoal, 0, 1.2*angularMaxSpeed, angleMargin);
    Displacement::publishCmdVel();
    /*if(Wz < 0.01){
        Displacement::setGoalReachedFlag(1);
    }*/
   
    /*}
    else if (goalReached.data) //TODO: Make the homing work 
    {
        Vx = -0.05;
        Displacement::publishCmdVel();
    }*/
}
/*
  *   The idea is to start muving in the direction of the nextPoint relative to base_link frame
  *   But, to give some preference to X direction of muvement(forward) because of reasons(in ARCO for example we have blind areas in the laterals)
  *   So we will give velocity v=(vx,vy) in the direction of the next point, but at the same time the robot will rotate to face the point and thus
  *   the mouvement will finally be in x direction. 
  *   Also, we will use differents top speed for x and y because is more dangerous to move in y direction than x. 
*/
void Displacement::moveHolon(double finalYaw){

    v = Displacement::getVel(linearMaxSpeed,b,dist2GlobalGoal,0);

    //if(angle2NextPoint > M_PI/3)
    //    v/=2;
    
    Vx = cos(angle2NextPoint) * v; // /(fabs(angle2NextPoint)/M_PI_2*3);
    Vy = sin(angle2NextPoint) * v; // /(fabs(angle2NextPoint)/M_PI_2*3);
    ROS_WARN("Theta: %.2f\t Vx %.2f\tVy %.2f", angle2NextPoint,Vx, Vy);
    /*if (dist2GlobalGoal < distMargin * 1.5)
    {
        Vx /= 2;
        Vy /= 2;
    }*/
    
    Displacement::someoneInFront(Vx,Vy);
    
    if (fabs(angle2NextPoint) > d2rad(angleMargin))
    {
        //Wz = angularMaxSpeed * angle2NextPoint / fabs(angle2NextPoint);

        Wz = Displacement::getVel(angularMaxSpeed, a, angle2NextPoint, 0);
        /*if (fabs(angle2NextPoint) < 2 * d2rad(angleMargin))
        {
            Wz = (angularMaxSpeed / 3) * angle2NextPoint / fabs(angle2NextPoint);
        }*/
    }

    //If we are close to the goal, let's start orientate the robot towards the final yaw
    if (dist2GlobalGoal < startOrientateDist)
        Displacement::setRobotOrientation(finalYaw, 0, 0, angularMaxSpeed, 10);

}
void Displacement::moveNonHolon()
{
    if (fabs(angle2NextPoint) > d2rad(angleMargin))
    {
        //Wz = 4 * angle2NextPoint * 180 / M_PI * (angularMaxSpeed - 0.15) / (180 - angleMargin) + angularMaxSpeed - 180 / (180 - angleMargin) * (angularMaxSpeed - 0.15);
        Wz = Displacement::getVel(angularMaxSpeed,a,angle2NextPoint,0);
        //ROS_WARN_THROTTLE(0.5,"%.2f",Wz);
    }
    else
    {
        Vx = Displacement::getVel(linearMaxSpeed,b,dist2GlobalGoal,0);
        Vy = 0;
        Wz = Displacement::getVel(angularMaxSpeed/2,a,angle2NextPoint,0);
        //Wz = 3 * angle2NextPoint;
    }
}

void Displacement::navigate()
{
    Displacement::refreshParams();
    //if(possible_to_move.data){
        //start == ros::Time::now();
        if (trajReceived && !goalReached.data && do_navigate)
        {
            Vx = 0;
            Vy = 0;
            Wz = 0;

            PoseStamp nextPoseBlFrame = Displacement::transformPose(nextPoint, "map", "base_link");
            angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);
            dist2NextPoint = Displacement::euclideanDistance(nextPoseBlFrame);

            globalGoalPose = Displacement::transformPose(globalGoal, "map", "base_link");
            dist2GlobalGoal = Displacement::euclideanDistance(globalGoalPose);
            angle2GlobalGoal = Displacement::getYawFromQuat(globalGoalPose.pose.orientation);


            if (dist2GlobalGoal < distMargin && !goalReached.data)
            {
                ROS_INFO_ONCE("Maniobra de aproximacion");
                Displacement::aproximateTo(&globalGoal, 1, 0);
            }else if (holonomic){
                Displacement::moveHolon(getYawFromQuat(globalGoal.pose.orientation));
            }else{
                Displacement::moveNonHolon();
            }
            Displacement::publishCmdVel();

        }
    /*}else{
        //Si no se puede mover, esperar 5s, si despues de 5s sigue sin poder, deberia moverse para intentar refrescar
        //el mapa
        ROS_INFO_THROTTLE(1,"No solution, so I wait the space to be clear");
        Displacement::publishZeroVelocity();
        if(start.toSec() - ros::Time::now().toSec() > 5){
            //dosomehtin
            Displacement::rotateToRefresh();
        }


    }*/
}
void Displacement::rotateToRefresh(){
    
}
/**
 * Inputs must be:
 * max: m/s or rad/s
 * var: m/s or rad
 *  
**/
float Displacement::getVel(float max, float exp_const, float var,int mode)
{
    switch (mode){
        case 0:
            return max*(1-exp(-exp_const*fabs(var)))*var/fabs(var);
        default:
            return 0;
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
void Displacement::computeDistanceToPeople(){

    dist2people.clear();
    
    PoseStamp bl_pose, person;

    bl_pose.header.frame_id = "base_link";
    bl_pose.header.seq = rand();
    bl_pose.header.stamp = ros::Time::now();

    bl_pose.pose.position.x = 0;
    bl_pose.pose.position.y  =0;
    bl_pose.pose.position.z = 0;

    bl_pose.pose.orientation.w = 1;
    bl_pose.pose.orientation.z = 0;


    bl_pose = transformPose(bl_pose,"base_link", "map");
    //ROS_WARN("base_link pose: [%.2f, %.2f]", bl_pose.pose.position.x, bl_pose.pose.position.y);
    pair<string, pair<float,float>> p;

    //ROS_WARN_THROTTLE(1,"Name\t Dist\t Angle\n");
    
    for(int i = 0; i < peopl.people.size(); i++){
        
        person.pose.position = peopl.people.at(i).position;
        p.first = peopl.people.at(i).name;
        p.second.first = euclideanDistance(bl_pose, person);

        p.second.second = atan2(person.pose.position.y, person.pose.position.x)/M_PI*180;
        
        dist2people.insert(p);
        //ROS_WARN_THROTTLE(1,"%s\t %.2f\t %.2f\n",p.first.c_str(), p.second.first,p.second.second);
    }

    
}
void Displacement::printPeople(){
    
    printf("\x1B[32m\n\tName  \t Distance\n");
    for(auto it = dist2people.begin(); it != dist2people.end(); ++it){
        printf("\x1B[32m\t  %s\t %.2f\n",it->first.c_str(), it->second.first);
    }
    printf("\x1B[0m");
}
//To check if anyone in the people list is in the direction mouvement cone +-40º centered in the
//velocity direction
bool Displacement::someoneInFront(){
   return Displacement::someoneInFront(Vx,Vy);
}
bool Displacement::someoneInFront(int vx, int vy){

    if(Vx != 0 || Vy != 0){
        float theta1 = atan2(vy,vx) /M_PI * 180;

        for(auto it = dist2people.begin(); it!=dist2people.end(); ++it){
            if(theta1 - alpha < -180){

            }
            if(theta1 + alpha > 180){

            }
            if(it->second.second > theta1-alpha && it->second.second < theta1+alpha ){
                ROS_INFO("%s : %.2f m, %.2f grados",it->first.c_str(),it->second.first,it->second.second);
                return true;
            }    
        }        
    }
    return false;
}
} // namespace Navigators
