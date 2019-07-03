/*
* Social Force Model Navigator Class: For holonomic robots
* Luis Merino 2019, UPO
*/
#include <navigator/sfmcontroller.hpp>
#include <navigator/securityMargin.hpp>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


namespace Navigators
{

void SFMNav::refreshParams()
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

    if (delta > traj_timeout)
    {
        trajReceived = false;
        SFMNav::publishZeroVelocity();
    }
   
}
//Displacement Class functions
SFMNav::SFMNav(ros::NodeHandle *n, SecurityMargin *margin_, tf2_ros::Buffer *tfBuffer_)
{
    nh = n;
    //Pointer to the security margin object created
    margin = margin_;
    tfBuffer = tfBuffer_;

    //Right now we will put the ARCO cmd vel topic but in the future it will selectable
    twist_pub = nh->advertise<geometry_msgs::Twist>("/idmind_motors/twist", 1);
    muving_state_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/muving_state", 1);
    goal_reached_pub = nh->advertise<std_msgs::Bool>("/trajectory_tracker/local_goal_reached", 1);
    goal_pub = nh->advertise<PoseStamp>("/move_base_simple/goal", 1);
    leds_pub = nh->advertise<std_msgs::UInt8MultiArray>("/idmind_sensors/set_leds", 1);

    //Advertise SFM related markers
    robot_markers_pub = nh->advertise<visualization_msgs::MarkerArray>("/sfm/markers/robot_forces", 1);

    nh->param("/nav_node/do_navigate", do_navigate, (bool)true);
    nh->param("/nav_node/holonomic", holonomic, (bool)true);
    nh->param("/nav_node/angular_max_speed", angularMaxSpeed, (float)0.5);
    nh->param("/nav_node/linear_max_speed", linearMaxSpeed, (float)0.3);
    nh->param("/nav_node/angle_margin", angleMargin, (float)15);
    nh->param("/nav_node/dist_margin", distMargin, (float)0.35);
    nh->param("/nav_node/a", a, (float)5);
    nh->param("/nav_node/b", b, (float)5);
    nh->param("/nav_node/traj_timeout", traj_timeout, (float)0.025);
    nh->param("/nav_node/start_orientate_dist", startOrientateDist, (float)0.025);
    nh->param("/nav_node/robot_base_frame", robot_frame, (string) "base_link");
    nh->param("/nav_node/world_frame", world_frame, (string) "map");


    //SFM params
    nh->param<double>("robot_radius",robot_radius,0.35);
    nh->param<double>("person_radius",person_radius,0.35);
    nh->param<double>("target_velocity",target_velocity,0.6);
    nh->param<double>("people_velocity",people_velocity,0.6);
    nh->param<double>("robot_velocity",robot_velocity,0.6);
    nh->param<double>("robot_max_lin_acc",robot_max_lin_acc,1.0);
    nh->param<double>("robot_max_ang_acc",robot_max_ang_acc,2.0);
    nh->param<double>("beta_v",beta_v,0.4);
    nh->param<double>("beta_y",beta_y,0.3);
    nh->param<double>("beta_d",beta_d,0.3);

    nh->param<double>("obstacle_distance_threshold",obstacle_distance_threshold,2.0);	
    nh->param<double>("naive_goal_time",naive_goal_time,2.0);
    nh->param<double>("goal_radius",goal_radius,0.25);

	//Initialize SFM. Just one agent (the robot)
	agents.resize(1);
	agents[0].desiredVelocity = robot_velocity;
	agents[0].radius = robot_radius;
	agents[0].cyclicGoals = false;
	agents[0].teleoperated = true;


    //Start flags values
    //Flags to publish
    possible_to_move.data = true;
    localGoalOcc.data = false;
    goalReached.data = false;
    //Flags for internal states
    homePublished = false;
    trajReceived = false;
    aproxComplete = false;

    tr0_catch = false;
    alpha = 30;
}




void SFMNav::occLocalGoalCb(const std_msgs::Bool::ConstPtr &msg)
{
	ROS_INFO("Local goal occupied received");
    localGoalOcc = *msg;
    if (localGoalOcc.data)
    {
        SFMNav::publishZeroVelocity();
    }
}


void SFMNav::impossibleMoveCb(const std_msgs::Bool::ConstPtr &msg)
{
    possible_to_move.data = !(msg->data);
}


void SFMNav::odomReceived(const nav_msgs::Odometry::ConstPtr& odom)
{
	ROS_INFO("Odom received");

	if (odom->header.frame_id != "/odom" && odom->header.frame_id !="odom") {
		ROS_INFO("Odometry frame is %s, it should be odom",odom->header.frame_id.c_str()); 
		return;
	}	

	//Update agent[0] (the robot) with odom. Our robot is omnidirectional
	agents[0].position.set(odom->pose.pose.position.x,odom->pose.pose.position.y); 
	agents[0].yaw = utils::Angle::fromRadian(tf::getYaw(odom->pose.pose.orientation));
	
	agents[0].linearVelocity = std::sqrt(odom->twist.twist.linear.x*odom->twist.twist.linear.x +odom->twist.twist.linear.y*odom->twist.twist.linear.y);
	agents[0].angularVelocity = odom->twist.twist.angular.z;
	
	//The velocity in the odom messages is in the robot local frame!!!
	geometry_msgs::Vector3 velocity;
	velocity.x = odom->twist.twist.linear.x;
	velocity.y = odom->twist.twist.linear.y;

	geometry_msgs::Vector3 localV = SFMNav::transformVector(velocity,robot_frame,"odom");
	agents[0].velocity.set(localV.x, localV.y);
 	//agents[0].velocity.set(odom->twist.twist.linear.x, odom->twist.twist.linear.y);
	


}

void SFMNav::poseReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{

	ROS_INFO("Pose received");
	
	//agents[0].position.set(pose->pose.pose.position.x,pose->pose.pose.position.y); 
	//agents[0].yaw = utils::Angle::fromRadian(tf::getYaw(pose->pose.pose.orientation));
	//agents[0].linearVelocity = std::sqrt(odom->twist.twist.linear.x*odom->twist.twist.linear.x +odom->twist.twist.linear.y*odom->twist.twist.linear.y);
	//agents[0].angularVelocity = odom->twist.twist.angular.z;	
	//agents[0].velocity.set(odom->twist.twist.linear.x, odom->twist.twist.linear.y);
	


}



void SFMNav::trackedPersonCb(const people_msgs::People::ConstPtr &people)
{

	ROS_INFO("People received");

    peopl = *people;


	//if (people->header.frame_id != "/odom" && people->header.frame_id !="odom") {
	//	ROS_INFO("People frame is %s, it should be odom",people->header.frame_id.c_str()); 
	//	return;
	//}


	//The SFM permits to define a person to accompany (the target)
	//and groups (through groupId). They are not considered by now
	target_index=0;
	targetFound=false;
	agents[0].groupId = -1;
	agents.resize(people->people.size()+1);

	//People are received in the global frame. We transform them to odom
	for (unsigned i=0; i< people->people.size(); i++) {

		geometry_msgs::Point point = SFMNav::transformPoint(people->people[i].position,world_frame,"odom");

		agents[i+1].position.set(point.x,point.y);

		geometry_msgs::Vector3 velocity;
		velocity.x = people->people[i].velocity.x;
		velocity.y = people->people[i].velocity.y;

		geometry_msgs::Vector3 localV = SFMNav::transformVector(velocity,world_frame,"odom");

		agents[i+1].yaw = utils::Angle::fromRadian(atan2(localV.y, localV.x));
		agents[i+1].velocity.set(localV.x, localV.y);
		agents[i+1].linearVelocity = agents[i+1].velocity.norm();
		agents[i+1].radius = person_radius;
		agents[i+1].teleoperated=false;
		//if (fabs(people->people[i].vel) < 0.05) {
		//	agents[i+1].velocity.set(0,0);
		//} 

		//The SFM requires a local goal for each agent. We will assume that the goal for people
		//depends on its current velocity
		agents[i+1].goals.clear();		
		sfm::Goal naiveGoal;
		naiveGoal.center =agents[i+1].position + naive_goal_time * agents[i+1].velocity;
		naiveGoal.radius = goal_radius;
		agents[i+1].goals.push_back(naiveGoal);
		if (false/*people->people[i].name == targetName*/) {
			targetFound = true;
			target_index=i+1;
			agents[i+1].desiredVelocity = target_velocity;
			agents[i+1].groupId = 0;
			agents[0].groupId = 0;
			targetPos = agents[i+1].position;
			targetVel = agents[i+1].velocity;
		} else {
			agents[i+1].desiredVelocity = people_velocity;
			agents[i+1].groupId = -1;
		}
	}
  
}


void SFMNav::trajectoryCb(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &trj)
{
	ROS_INFO("Trajectory received");
    trajReceived = true;
    ros::param::get("/nav_node/traj_timeout", traj_timeout);

    delta = ros::Time::now().toSec() - trj->header.stamp.toSec();

    //ROS_INFO("DELTA: %.3f\n now: %.3f",delta,ros::Time::now().toSec());

    nextPoint = trj->points[trj->points.size() > 1 ? 1 : 0];
}


//Used to calculate distance from base_link to global goal
void SFMNav::globalGoalCb(const geometry_msgs::PoseStampedConstPtr &globGoal_)
{
	ROS_INFO("Global Goal received");
    goalReached.data = false;
    aproxComplete = false;
    globalGoal = *globGoal_;
}


void SFMNav::setRobotOrientation(geometry_msgs::Quaternion q, bool goal, bool pub, float speed, float angleMargin_)
{
    SFMNav::setRobotOrientation(SFMNav::getYawFromQuat(q), goal, pub, speed, angleMargin_);
}


void SFMNav::setRobotOrientation(float finalYaw, bool goal, bool pub, float speed, float angleMargin_)
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

    robotPose = SFMNav::transformPose(robotPose, robot_frame, world_frame);
    float robotYaw = SFMNav::getYawFromQuat(robotPose.pose.orientation);

    if (robotYaw < 0)
        robotYaw += 360;
    if (finalYaw < 0)
        finalYaw += 360;

    float yawDif = finalYaw - robotYaw;

    if (fabs(yawDif) > 180)
        yawDif -= 360 * yawDif / fabs(yawDif);

    if (fabs(yawDif) > angleMargin_)
    {
        //This one works VERY WELL DONT TOUCH IT
        Wz = 2 * (yawDif * (speed - 0.1) / (180 - angleMargin_) + speed - 180 / (180 - angleMargin_) * (speed - 0.1));
    }
    else if (goal)
    {
        SFMNav::setGoalReachedFlag(1);
        traj_timeout = std::numeric_limits<float>::max();
    }

    if (pub)
        SFMNav::publishCmdVel();
}



void SFMNav::aproximateTo(geometry_msgs::PoseStamped *pose, bool isGoal, bool isHome)
{

    geometry_msgs::PoseStamped p = transformPose(*pose, world_frame, robot_frame);

    Vx = p.pose.position.x;
    Vy = p.pose.position.y;

    SFMNav::setRobotOrientation(getYawFromQuat(pose->pose.orientation), isGoal, 0, 1.7 * angularMaxSpeed, angleMargin);
    SFMNav::publishCmdVel();
}





void SFMNav::navigate()
{
    //SFMNav::refreshParams();

    sfm::CmdVelProvider cmdVelProvider(obstacle_distance_threshold,robot_velocity ,robot_max_lin_acc,robot_max_ang_acc, beta_v,  beta_y,  beta_d);

    if (trajReceived && !goalReached.data && do_navigate && !localGoalOcc.data && possible_to_move.data)
    {
        if (aproxComplete)
        {
            aproxComplete = false;
        }
        Vx = 0;
        Vy = 0;
        Wz = 0;

	//Local goal for the SFM for the robot. We transform it to odom
	PoseStamp nextGoalOdom = SFMNav::transformPose(nextPoint, world_frame, "odom");

        PoseStamp nextPoseBlFrame = SFMNav::transformPose(nextPoint, world_frame, robot_frame);
        angle2NextPoint = atan2(nextPoseBlFrame.pose.position.y, nextPoseBlFrame.pose.position.x);
        dist2NextPoint = SFMNav::euclideanDistance(nextPoseBlFrame);

        globalGoalPose = SFMNav::transformPose(globalGoal, world_frame, robot_frame);
        dist2GlobalGoal = SFMNav::euclideanDistance(globalGoalPose);
        angle2GlobalGoal = SFMNav::getYawFromQuat(globalGoalPose.pose.orientation);

	ROS_INFO("Navigate");

        if (dist2GlobalGoal < distMargin && !goalReached.data)
        {
            ROS_INFO_ONCE("Maniobra de aproximacion");
            SFMNav::aproximateTo(&globalGoal, 1, 0);
        }
        else
        {
           	ROS_INFO("SFM:%f,%f ",nextGoalOdom.pose.position.x, nextGoalOdom.pose.position.y);
		agents[0].goals.clear();
		robot_local_goal.center.set(nextGoalOdom.pose.position.x,nextGoalOdom.pose.position.y);

		robot_local_goal.radius = goal_radius;
		agents[0].goals.push_back(robot_local_goal);

		//Compute resultant forces for all agents
		sfm::SFM.computeForces(agents);
		
			cmdVelProvider.compute(agents[0],agents[target_index],targetFound,!agents[0].antimove,agents[0].params.relaxationTime);
			//cmd_vel_pub.publish(cmdVelProvider.getCommand());


			
			publishForces();
			//publishPath(goalProvider);
			//publishGoal();
			//publishTrajectories(cmdVelProvider);
			//publishScan();
			//publishTarget();			
			
		
       }
       // SFMNav::publishCmdVel();
    }
    if (goalReached.data && !aproxComplete)
    {
        ROS_WARN("Ejecutando acercamiento para carga marcha atras");
        if (!tr0_catch)
        {
            try
            {
                tr0 = tfBuffer->lookupTransform("base_link", "odom", ros::Time(0));
                tr0_catch = true;
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
                tr1 = tfBuffer->lookupTransform("base_link", "odom", ros::Time(0));
                dlt.x = tr1.transform.translation.x - tr0.transform.translation.x;
                dlt.y = tr1.transform.translation.y - tr0.transform.translation.y;
                vel.linear.x = SFMNav::getVel(0.2, 2, 1 - sqrtf(dlt.x * dlt.x + dlt.y * dlt.y));
                ROS_ERROR("Aprox vel: %.2f ; Dist: %.2f", vel.linear.x, sqrtf(dlt.x * dlt.x + dlt.y * dlt.y));
                if (sqrtf(dlt.x * dlt.x + dlt.y * dlt.y) < 1)
                {
                   // twist_pub.publish(vel);
                }
                else
                {
                    aproxComplete = true;
                    tr0_catch = false;
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
float SFMNav::getVel(float max, float exp_const, float var)
{ 
    return max * (1 - exp(-exp_const * fabs(var))) * var / fabs(var); 
}


bool SFMNav::hasFinished()
{
    return goalReached.data;
}


void SFMNav::publishZeroVelocity()
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


void SFMNav::setGoalReachedFlag(bool status_)
{
    goalReached.data = false;
    if (status_)
    {
        goalReached.data = true;
        SFMNav::publishZeroVelocity();
        goal_reached_pub.publish(goalReached);
    }
}


void SFMNav::publishCmdVel()
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
        SFMNav::publishZeroVelocity();
    }
   
    //goal_reached_pub.publish(goalReached);
}

PoseStamp SFMNav::transformPose(trajectory_msgs::MultiDOFJointTrajectoryPoint point, std::string from, std::string to)
{
    PoseStamp pose;
    pose.header.frame_id = from;
    pose.header.seq = rand();
    pose.header.stamp = ros::Time(0);

    pose.pose.orientation = point.transforms[0].rotation;

    pose.pose.position.x = point.transforms[0].translation.x;
    pose.pose.position.y = point.transforms[0].translation.y;
    pose.pose.position.z = point.transforms[0].translation.z;
    return SFMNav::transformPose(pose, from, to);
}


PoseStamp SFMNav::transformPose(PoseStamp originalPose, std::string from, std::string to)
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

geometry_msgs::Point SFMNav::transformPoint(geometry_msgs::Point point, std::string from, std::string to)
{

    geometry_msgs::Point nextPoint;

    geometry_msgs::PointStamped p;
    geometry_msgs::PointStamped np;

    p.header.frame_id = from;

    //we'll just use the most recent transform available for our simple example
    p.header.stamp = ros::Time(0);

  	//just an arbitrary point in space
    p.point.x = point.x;
    p.point.y = point.y;
    p.point.z = 0.0;

    try
    {
	tfBuffer->transform(p,np, to);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("No transform %s", ex.what());
    }

    nextPoint.x = np.point.x;
    nextPoint.y = np.point.y;
    nextPoint.z = 0.0;

    return nextPoint;
}

geometry_msgs::Vector3 SFMNav::transformVector(geometry_msgs::Vector3 vector, std::string from, std::string to)
{

    geometry_msgs::Vector3 nextVector;

    geometry_msgs::Vector3Stamped v;
    geometry_msgs::Vector3Stamped nv;

    v.header.frame_id = from;

    //we'll just use the most recent transform available for our simple example
    v.header.stamp = ros::Time(0);

  	//just an arbitrary point in space
    v.vector.x = vector.x;
    v.vector.y = vector.y;
    v.vector.z = 0.0;

    try
    {
	tfBuffer->transform(v,nv, to);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("No transform %s", ex.what());
    }

    nextVector.x = nv.vector.x;
    nextVector.y = nv.vector.y;
    nextVector.z = 0.0;

    return nextVector;
}


float SFMNav::getYawFromQuat(geometry_msgs::Quaternion quat)
{
    double r, p, y;
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 M(q);
    M.getRPY(r, p, y);

    return y / M_PI * 180;
}





void SFMNav::laserReceived(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	ROS_INFO("Laser received");
	
	if (scan->header.frame_id != "/base_link" && scan->header.frame_id !="base_link") {
		ROS_ERROR("Laser frame is %s, it should be base_link",scan->header.frame_id.c_str()); 
		return;
	}

	utils::Angle alpha = agents[0].yaw + utils::Angle::fromRadian(scan->angle_min);
	utils::Angle angle_inc = utils::Angle::fromRadian(scan->angle_increment);

	agents[0].obstacles1.clear();
	for (unsigned i=0; i<scan->ranges.size();i++) {
		if (!std::isnan(scan->ranges[i]) && scan->ranges[i]<obstacle_distance_threshold) {
			agents[0].obstacles1.emplace_back(scan->ranges[i]*alpha.cos(),scan->ranges[i]*alpha.sin());
		}
		alpha+=angle_inc;
	}	


}


void SFMNav::publishForceMarker(unsigned index, const std_msgs::ColorRGBA& color, 
					const utils::Vector2d& force, 
					visualization_msgs::MarkerArray& markers) 
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "odom";
	marker.header.stamp = ros::Time::now();
	marker.ns = "robot_forces";
	marker.id = index;
	marker.action = force.norm()>1e-4 ?0:2;
	marker.color = color;
	marker.lifetime = ros::Duration(1.0);
	marker.scale.x = std::max(1e-4,force.norm());
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.pose.position.x = agents[0].position.getX();
	marker.pose.position.y = agents[0].position.getY();
	marker.pose.position.z = 0;
	marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,force.angle().toRadian());
	markers.markers.push_back(marker);
}

std_msgs::ColorRGBA SFMNav::getColor(double r, double g, double b, double a)
{
	std_msgs::ColorRGBA color;
	color.r = r;
	color.g = g;
	color.b = b;
	color.a = a;
	return color;
}

void SFMNav::publishForces()
{
	visualization_msgs::MarkerArray markers;
	publishForceMarker(0,getColor(0,0,1,1),agents[0].forces.obstacleForce,markers);
	publishForceMarker(1,getColor(0,1,1,1),agents[0].forces.socialForce,markers);	
	publishForceMarker(2,getColor(0,1,0,1),agents[0].forces.groupForce,markers);	
	publishForceMarker(3,getColor(1,0,0,1),agents[0].forces.desiredForce,markers);	
	publishForceMarker(4,getColor(1,1,1,1),agents[0].forces.globalForce,markers);	
	publishForceMarker(5,getColor(1,1,0,1),agents[0].velocity,markers);	
	robot_markers_pub.publish(markers);


	/*teresa_wsbs::Info forces;
	forces.header.frame_id = "/odom";
	forces.header.stamp = ros::Time::now();
	

	forces.status = (uint8_t)state;
	forces.mode = (uint8_t)controller_mode;
	forces.target_detected = targetFound;
	forces.target_id = targetId;
	if (targetFound) {
		forces.target_pose.x = agents[target_index].position.getX();
		forces.target_pose.y = agents[target_index].position.getY();
		forces.target_pose.theta = agents[target_index].yaw.toRadian();
		forces.target_lin_vel = agents[target_index].linearVelocity;
		forces.target_group_force.x =  agents[target_index].forces.groupForce.getX();
		forces.target_group_force.y =  agents[target_index].forces.groupForce.getY();	
		forces.target_group_vis_force.x = agents[target_index].forces.groupGazeForce.getX();
		forces.target_group_vis_force.y = agents[target_index].forces.groupGazeForce.getY();
		forces.target_group_att_force.x = agents[target_index].forces.groupCoherenceForce.getX();
		forces.target_group_att_force.y = agents[target_index].forces.groupCoherenceForce.getY();
		forces.target_group_rep_force.x = agents[target_index].forces.groupRepulsionForce.getX();
		forces.target_group_rep_force.y = agents[target_index].forces.groupRepulsionForce.getY();
	}
	forces.robot_pose.x = agents[0].position.getX();
	forces.robot_pose.y = agents[0].position.getY();
	forces.robot_pose.theta = agents[0].yaw.toRadian();

	forces.robot_lin_vel = agents[0].linearVelocity;
	forces.robot_ang_vel = agents[0].angularVelocity;

	forces.robot_antimove =  agents[0].antimove;
	forces.robot_local_goal.x = agents[0].goals.front().center.getX();
	forces.robot_local_goal.y = agents[0].goals.front().center.getY();
	

	forces.robot_global_force.x = agents[0].forces.globalForce.getX();
	forces.robot_global_force.y = agents[0].forces.globalForce.getY();
	forces.robot_desired_force.x = agents[0].forces.desiredForce.getX();
	forces.robot_desired_force.y = agents[0].forces.desiredForce.getY();
	forces.robot_obstacle_force.x = agents[0].forces.obstacleForce.getX();
	forces.robot_obstacle_force.y = agents[0].forces.obstacleForce.getY();
	forces.robot_social_force.x = agents[0].forces.socialForce.getX();
	forces.robot_social_force.y = agents[0].forces.socialForce.getY();
	forces.robot_group_force.x =  agents[0].forces.groupForce.getX();
	forces.robot_group_force.y =  agents[0].forces.groupForce.getY();	
	forces.robot_group_vis_force.x = agents[0].forces.groupGazeForce.getX();
	forces.robot_group_vis_force.y = agents[0].forces.groupGazeForce.getY();
	forces.robot_group_att_force.x = agents[0].forces.groupCoherenceForce.getX();
	forces.robot_group_att_force.y = agents[0].forces.groupCoherenceForce.getY();
	forces.robot_group_rep_force.x = agents[0].forces.groupRepulsionForce.getX();
	forces.robot_group_rep_force.y = agents[0].forces.groupRepulsionForce.getY();
	forces.robot_vref.x = agents[0].velocity.getX();
	forces.robot_vref.y = agents[0].velocity.getY();

	forces_pub.publish(forces);
	*/
}


} // namespace Navigators
