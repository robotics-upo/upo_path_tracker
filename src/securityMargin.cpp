
#include <navigator/securityMargin.hpp>

SecurityMargin::SecurityMargin(ros::NodeHandle *n, tf2_ros::Buffer *tfBuffer_)
{
    tfBuffer = tfBuffer_;
    SecurityMargin::setParams(n);
   
}
void SecurityMargin::buildSelected(){

    
   
    switch (secMode)
    {
    case 0:
        ROS_INFO("Building in mode 0");
        SecurityMargin::buildArrays();
        ROS_INFO("Finished");
        break;
    case 1:
        ROS_INFO("Building in mode 1");
        SecurityMargin::buildArraysSquare2(0);
        SecurityMargin::buildArraysSquare2(1);
        ROS_INFO("Finished");
        break;
    case 2:
        ROS_INFO("Building in mode 2");
        SecurityMargin::buildElliptic();
        ROS_INFO("Finished");
        break;
    default:
        ROS_INFO("Wrong mode: %d", secMode);
        break;
    }
    
}
void SecurityMargin::laser1Callback(const sensor_msgs::LaserScanConstPtr &scan) //Front
{   
    //ROS_INFO("Testing laser 1: %.2f", scan->ranges.at(40));
    laser1CPtr = scan;
    if (!laser1Got)
    {
        
        frontLaserArrayMsgLen = scan->ranges.size();
        laser1Got = true;
        if (secMode == 0)
        {
            float laserSecurityAngleFront_;
            nh->param("nav_node/laser_security_angle_front", laserSecurityAngleFront_, (float)15);
            laserSecurityAngleFront = ceil(laserSecurityAngleFront_ * (frontLaserArrayMsgLen / 180));
            
        }
    }
    if (laser2Got && !lasersGot){
        lasersGot = true;
        SecurityMargin::buildSelected();
    }
    
}
void SecurityMargin::laser2Callback(const sensor_msgs::LaserScanConstPtr &scan) //Rear
{
    laser2CPtr = scan;
    if (!laser2Got)
    {
        
        backLaserArrayMsgLen = scan->ranges.size();
        laser2Got = true;
        if (secMode == 0)
        {
            float laserSecurityAngleBack_;
            nh->param("nav_node/laser_security_angle_back", laserSecurityAngleBack_, (float)15);
            laserSecurityAngleBack = ceil(laserSecurityAngleBack_ * (backLaserArrayMsgLen / 180));
        }
    }

    if (laser1Got && !lasersGot){
        lasersGot = true;
        SecurityMargin::buildSelected();
    }
        
}
void SecurityMargin::aproachManCb(const std_msgs::Bool::ConstPtr &msg){
    aproaching_status=*msg;
}
void SecurityMargin::goalReachedCb(const std_msgs::Bool::ConstPtr &msg){
    goal_reached = *msg;
}
void SecurityMargin::refreshParams()
{
    float laserSecurityAngleBack_, laserSecurityAngleFront_;
    if(secMode == 0){
    ros::param::get("nav_node/f_front", f1);
    }else{
        ros::param::get("nav_node/f", f1);
    }

    ros::param::get("nav_node/f_back", f2);
    //ros::param::get("nav_node/inner_radius_front", innerSecDistFront);
    //ros::param::get("nav_node/outer_radius_front", extSecDistFront);
    //ros::param::get("nav_node/inner_radius_back", innerSecDistBack);
    //ros::param::get("nav_node/outer_radius_back", extSecDistBack);
    ros::param::get("nav_node/delta_d", delta_d);
    ros::param::get("nav_node/margin_x", margin_x);
    ros::param::get("nav_node/margin_y", margin_y);
    ros::param::get("nav_node/count1", count1);
    ros::param::get("nav_node/count2", count2);
    
    laserSecurityAngleFront = ceil(laserSecurityAngleFront_ * (frontLaserArrayMsgLen / 180));
    laserSecurityAngleBack = ceil(laserSecurityAngleBack_ * (backLaserArrayMsgLen / 180));
}
void SecurityMargin::dist2GoalCb(const std_msgs::Float32::ConstPtr &msg){
    dist2goal=*msg;
}
void SecurityMargin::setParams(ros::NodeHandle *n)
{
    nh = n;
    //Markers publishers

    nh->param("nav_node/security_mode", secMode, (int)1);
    nh->param("nav_node/publish_markers", pubMarkers, (bool)1);
    nh->param("nav_node/front_laser_link_frame_id", front_laser_link_frame, (string) "front_laser_link");
    string stop_topic;
    nh->param("nav_node/security_stop_topic", stop_topic, (string) "/security_stop");
    stop_pub = nh->advertise<std_msgs::Bool>(stop_topic, 0);
    nh->param("nav_node/count1", count1, (int)25);
    nh->param("nav_node/count2", count2, (int)200);
    aproach_man_sub = nh->subscribe<std_msgs::Bool>("/trajectory_tracker/aproach_manoeuvre",1,boost::bind(&SecurityMargin::aproachManCb,this,_1));
    dist2goal_sub = nh->subscribe<std_msgs::Float32>("/dist2goal",1,boost::bind(&SecurityMargin::dist2GoalCb,this,_1));

    goal_reached_sub = nh->subscribe<std_msgs::Bool>("/trajectory_tracker/local_goal_reached",1,boost::bind(&SecurityMargin::goalReachedCb,this,_1));
    //ROS_INFO("Security Mode: %d", secMode);
    //ROS_INFO("Front laser msg lenght: %d",frontLaserArrayMsgLen);
    //ROS_INFO("Back laser msg lenght: %d",backLaserArrayMsgLen);

    switch (secMode)
    {

    case 0:
        nh->param("nav_node/f_front", f1, (float)1);
        nh->param("nav_node/inner_radius_front", innerSecDistFront, (float)0.6);
        nh->param("nav_node/outer_radius_front", extSecDistFront, (float)1);
        nh->param("nav_node/only_front", onlyFront, (bool)0);
        nh->param("nav_node/front_laser_link_frame_id", front_laser_link_frame, (string) "front_laser_link");
        nh->param("nav_node/laser_front_topic", laser1_topic, (string)"/scanFront");
        //ROS_INFO("Front f factor: %.2f", f1);
        //ROS_INFO("Only front: %d", onlyFront);
        //ROS_INFO("Inner radius front: %.2f", innerSecDistFront);
        //ROS_INFO("Outer radius front: %.2f", extSecDistFront);
        //ROS_INFO("Front laser link frame: %s", front_laser_link_frame.c_str());
        if(!onlyFront){
            nh->param("nav_node/back_laser_link_frame_id", back_laser_link_frame, (string) "back_laser_link");
            nh->param("nav_node/inner_radius_back", innerSecDistBack, (float)0.6);
            nh->param("nav_node/outer_radius_back", extSecDistBack, (float)1);
            nh->param("nav_node/f_back", f2, (float)1.6);
            nh->param("nav_node/laser_back_topic", laser2_topic, (string)"/scanBack");
            //ROS_INFO("Rear f factor: %.2f", f2);
            //ROS_INFO("Back laser link frame id: %s", back_laser_link_frame.c_str());
            //ROS_INFO("Inner radius back: %.2f", innerSecDistBack);
            //ROS_INFO("Outer radius back: %.2f", extSecDistBack);
            laser2_sub = nh->subscribe<sensor_msgs::LaserScan> (laser2_topic.c_str(), 1, boost::bind(&SecurityMargin::laser2Callback,this, _1));
            laser2Got=false;
        }else{
            laser2Got=true;
            //ROS_INFO("HEEE");
        }
        break;
    case 1:
        onlyFront = true;
        laser2Got=true;
        nh->param("nav_node/robot_base_frame", base_link_frame, (string) "base_link");
        nh->param("nav_node/delta_d", delta_d, (float)0.15);
        nh->param("nav_node/margin_x", margin_x, (float)0.5);
        nh->param("nav_node/margin_y", margin_y, (float)0.5);
        nh->param("nav_node/full_laser_topic", laser1_topic, (string)"/scanMulti");
        ROS_INFO("Robot base frame: %s", base_link_frame.c_str());
        ROS_INFO("Delta d: %.2f",delta_d);
        ROS_INFO("Margin x: %.2f", margin_x);
        ROS_INFO("Margin y: %.2f", margin_y);
        break;
    case 2:
        onlyFront = true;
        laser2Got=true;
        nh->param("nav_node/robot_base_frame", base_link_frame, (string) "base_link");
        nh->param("nav_node/f", f1, (float)1);
        nh->param("nav_node/inner_radius", innerSecDistFront, (float)0.3);
        nh->param("nav_node/outer_radius", extSecDistFront, (float)0.45);
        nh->param("nav_node/full_laser_topic", laser1_topic, (string)"/scanMulti");
        ROS_INFO("Robot base frame: %s", base_link_frame.c_str());
        ROS_INFO("F factor: %.2f", f1);
        ROS_INFO("Inner radius: %.2f", innerSecDistFront);
        ROS_INFO("Outer radius: %.2f ", extSecDistFront);
        break;
    default:
        ROS_ERROR("Security Margin: Wrong mode, %d doesn't exist", secMode);
        break;
    }
    laser1_sub = nh->subscribe<sensor_msgs::LaserScan> (laser1_topic.c_str(), 1, boost::bind(&SecurityMargin::laser1Callback,this, _1));
    

    //Control flags
    isInsideDangerousArea1 = false;
    securityAreaOccup1 = false;

    isInsideDangerousArea2 = false;
    securityAreaOccup2 = false;

    paramsConfigured = true;

    red1 = false;
    red2 = false;

    laser1Got = false;
    lasersGot = false;

    aproximating = false;
    tr0_catch = false;

    goingAway = false;
    goal_reached.data = false;
    red.a = 1;
    red.b = 0;
    red.g = 0;
    red.r = 1;

    green.a = 1;
    green.g = 1;
    green.b = 0;
    green.r = 0;
    cnt=0;
    cnt2=0;
    if (pubMarkers)
    {
        marker_fr_1_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersFr1", 0);
        marker_fr_2_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersFr2", 0);
        marker_rr_1_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersRr1", 0);
        marker_rr_2_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersRr2", 0);
        if (secMode == 0)
        {
            markerIntFr.header.frame_id = front_laser_link_frame;
        }
        else
        {
            markerIntFr.header.frame_id = base_link_frame;
        }
        markerIntFr.header.stamp = ros::Time();
        markerIntFr.ns = "security_perimeter";
        markerIntFr.id = rand();
        markerIntFr.type = visualization_msgs::Marker::LINE_STRIP;
        markerIntFr.action = visualization_msgs::Marker::ADD;
        markerExtBack.lifetime = ros::DURATION_MAX;
        markerIntFr.scale.x = 0.025;
        markerIntFr.color = green;

        if (secMode == 0)
        {
            markerExtFr.header.frame_id = front_laser_link_frame;
        }
        else
        {
            markerExtFr.header.frame_id = base_link_frame;
        }

        markerExtFr.header.stamp = ros::Time();
        markerExtFr.ns = "security_perimeter";
        markerExtFr.id = rand();
        markerExtFr.type = visualization_msgs::Marker::LINE_STRIP;
        markerExtFr.action = visualization_msgs::Marker::ADD;
        markerExtBack.lifetime = ros::DURATION_MAX;
        markerExtFr.scale.x = 0.025;
        markerExtFr.color = green;

        if (!onlyFront)
        {
            markerIntBack.header.frame_id = back_laser_link_frame;
            markerIntBack.header.stamp = ros::Time();
            markerIntBack.ns = "security_perimeter";
            markerIntBack.id = rand();
            markerIntBack.type = visualization_msgs::Marker::LINE_STRIP;
            markerIntBack.action = visualization_msgs::Marker::ADD;
            markerExtBack.lifetime = ros::DURATION_MAX;
            markerIntBack.scale.x = 0.025;
            markerIntBack.color = green;

            markerExtBack.header.frame_id = back_laser_link_frame;
            markerExtBack.header.stamp = ros::Time();
            markerExtBack.ns = "security_perimeter";
            markerExtBack.id = rand();
            markerExtBack.type = visualization_msgs::Marker::LINE_STRIP;
            markerExtBack.action = visualization_msgs::Marker::ADD;
            markerExtBack.lifetime = ros::DURATION_MAX;
            markerExtBack.scale.x = 0.025;
            markerExtBack.color = green;
        }
    }
}
/* Mode 1

x horizontal, y vertical en el laser link
 
    Recta 1      x1
________________
                |  y1
  Laser   |_    |   R
  Link          |   E
                |   C
                |   T
                |   A
                |   2
                |
________________|   y2
    Recta 3


   ! Recta 1: y = y1
   !          x = [ y1/tan(angle_max): x1]

   ! Recta 2: y = [ x*tan(y1/x1) : x*tan(y2/x1) ]
   !          x = x1

   ! Recta 3: y = y2 
   !          x = [ y/tan(y2/x1) : y/tan(angle_min) ]
   

*/
void SecurityMargin::buildArraysSquare2( bool ext)
{
   if(!ext){

    a1.clear();
    a2.clear();
    secArrayFr.clear();

    p1.first = ext ? margin_x + delta_d : margin_x;
    p1.second = 0;
    
    p2.first = ext ? margin_x + delta_d : margin_x;
    p2.second = ext ? margin_y + delta_d : margin_y;
    

    L1 = p1.first;
    //ROS_INFO("l1: %f",L1);
    L2 = ext ? margin_y + delta_d : margin_y;
    //ROS_INFO("l2: %f",L2);
    L = 4 * L1 + 4 * L2;
    //ROS_INFO("L: %f",L);
    n1 = floor(frontLaserArrayMsgLen * L1 / L);
    n2 = floor(frontLaserArrayMsgLen * L2 / L);
    //ROS_WARN("N1: %d \t N2: %d",n1,n2);
    incr1 = L1 / n1;
    incr2 = L2 / n2;
    //ROS_INFO("P 1: [%.2f, %.2f]", p1.first,p1.second);
    //ROS_INFO("P 2: [%.2f, %.2f]", p2.first,p2.second);
    //ROS_INFO("n1: %d, n2: %d, L1 %f, L2 %f, L %f, incr1 %f, incr2 %f, msg len: %d  ", n1, n2, L1,L2,L,incr1,incr2, frontLaserArrayMsgLen);
    
    //Sec de push_back: L1,L2,L2,L1,L1,L2,L2,L1
    float value;
    for (int i = 0; i < n1; i++){
        value = sqrtf(p1.first * p1.first + i * incr1 * i * incr1);
        a1.push_back(value);
        //ROS_INFO("%.2f",value);
    }
    for (int i = 0; i < n2; i++){
        value=sqrtf(p2.second * p2.second + (p2.first - i * incr2) * (p2.first - i * incr2));
        a2.push_back(value);
         //ROS_INFO("%.2f",value);
    }
    for (int i = 0; i < n1; i++){
        value = a1.at(i);
        secArrayFr.push_back(value);
    }

    for (int i = 0; i < n2; i++){
        value = a2.at(i);
        //ROS_INFO("VALUE: %f",value);
        secArrayFr.push_back(value);
    }
    for (int i = n2; i > 0; i--){
        value = a2.at(i-1);
        //ROS_INFO("VALUE: %f",value);
        secArrayFr.push_back(value);
    }
    for (int i = n1; i > 0; i--){
        value = a1.at(i-1);
        //ROS_INFO("VALUE: %f",value);
        secArrayFr.push_back(value);
    }
    for (int i = 0; i < n1; i++){
        value = a1.at(i);
        //ROS_INFO("VALUE: %f",value);
        secArrayFr.push_back(value);
    }
    for (int i = 0; i < n2; i++){
        value = a2.at(i);
        //ROS_INFO("VALUE: %f",value);
        secArrayFr.push_back(value);
        
    }
    for (int i = n2; i > 0; i--){
        value = a2.at(i-1);
        //ROS_INFO("VALUE: %f",value);
        secArrayFr.push_back(value);
    }
    for (int i = n1; i > 0; i--){
        value = a1.at(i-1);
        //ROS_INFO("VALUE: %f",value);
        secArrayFr.push_back(value);
    }
    //ROS_INFO("Array size: %d", secArrayFr.size());
    int arrsize = secArrayFr.size();
    while (arrsize < frontLaserArrayMsgLen){
        
        secArrayFr.push_back(0);
        arrsize++;
        //ROS_INFO("pum");
    }
    markerExtFr.points.clear();
    geometry_msgs::Point p;
    p.x = p2.first;
    p.y = p2.second;
    p.z = 0.2;  
    markerExtFr.points.push_back(p);
    p.y *= -1;
    markerExtFr.points.push_back(p);
    p.x *= -1;
    markerExtFr.points.push_back(p);
    p.y *= -1;
    markerExtFr.points.push_back(p);
    p.x *= -1;
    markerExtFr.points.push_back(p);
    for(int i =  0; i < frontLaserArrayMsgLen; i++){
        
        //if(secArrayFr.at(i) != 0){
        //    ROS_WARN("%d",i);
        //    ROS_INFO("%.2f",secArrayFr.at(i));  
        //}
    }
   }else{

    a1.clear();
    a2.clear();
    secArrayExtFr.clear();

    p1.first = ext ? margin_x + delta_d : margin_x;
    p1.second = 0;
    
    p2.first = ext ? margin_x + delta_d : margin_x;
    p2.second = ext ? margin_y + delta_d : margin_y;
    

    L1 = p1.first;
    //ROS_INFO("l1: %f",L1);
    L2 = ext ? margin_y + delta_d : margin_y;
    //ROS_INFO("l2: %f",L2);
    L = 4 * L1 + 4 * L2;
    //ROS_INFO("L: %f",L);
    n1 = floor(frontLaserArrayMsgLen * L1 / L);
    n2 = floor(frontLaserArrayMsgLen * L2 / L);
    //ROS_WARN("N1: %d \t N2: %d",n1,n2);
    incr1 = L1 / n1;
    incr2 = L2 / n2;
    //ROS_INFO("P 1: [%.2f, %.2f]", p1.first,p1.second);
    //ROS_INFO("P 2: [%.2f, %.2f]", p2.first,p2.second);
    //ROS_INFO("n1: %d, n2: %d, L1 %f, L2 %f, L %f, incr1 %f, incr2 %f, msg len: %d  ", n1, n2, L1,L2,L,incr1,incr2, frontLaserArrayMsgLen);
    
    //Sec de push_back: L1,L2,L2,L1,L1,L2,L2,L1
    float value;
    for (int i = 0; i < n1; i++){
        value = sqrtf(p1.first * p1.first + i * incr1 * i * incr1);
        a1.push_back(value);
        //ROS_INFO("%.2f",value);
    }
    for (int i = 0; i < n2; i++){
        value=sqrtf(p2.second * p2.second + (p2.first - i * incr2) * (p2.first - i * incr2));
        a2.push_back(value);
         //ROS_INFO("%.2f",value);
    }
    for (int i = 0; i < n1; i++){
        value = a1.at(i);
        secArrayExtFr.push_back(value);
    }

    for (int i = 0; i < n2; i++){
        value = a2.at(i);
        //ROS_INFO("VALUE: %f",value);
        secArrayExtFr.push_back(value);
    }
    for (int i = n2; i > 0; i--){
        value = a2.at(i-1);
        //ROS_INFO("VALUE: %f",value);
        secArrayExtFr.push_back(value);
    }
    for (int i = n1; i > 0; i--){
        value = a1.at(i-1);
        //ROS_INFO("VALUE: %f",value);
        secArrayExtFr.push_back(value);
    }
    for (int i = 0; i < n1; i++){
        value = a1.at(i);
        //ROS_INFO("VALUE: %f",value);
        secArrayExtFr.push_back(value);
    }
    for (int i = 0; i < n2; i++){
        value = a2.at(i);
        //ROS_INFO("VALUE: %f",value);
        secArrayExtFr.push_back(value);
        
    }
    for (int i = n2; i > 0; i--){
        value = a2.at(i-1);
        //ROS_INFO("VALUE: %f",value);
        secArrayExtFr.push_back(value);
    }
    for (int i = n1; i > 0; i--){
        value = a1.at(i-1);
        //ROS_INFO("VALUE: %f",value);
        secArrayExtFr.push_back(value);
    }
    //ROS_INFO("Array size: %d", secArrayExtFr.size());
    int arrsize = secArrayExtFr.size();
    while (arrsize < frontLaserArrayMsgLen){
        
        secArrayExtFr.push_back(0);
        arrsize++;
        //ROS_INFO("pum");
    }
    markerIntFr.points.clear();
    geometry_msgs::Point p;
    p.x = p2.first;
    p.y = p2.second;
    p.z = 0.2;
    markerIntFr.points.push_back(p);
    p.y *= -1;
    markerIntFr.points.push_back(p);
    p.x *= -1;
    markerIntFr.points.push_back(p);
    p.y *= -1;
    markerIntFr.points.push_back(p);
    p.x *= -1;
    markerIntFr.points.push_back(p);
    for(int i =  0; i < frontLaserArrayMsgLen; i++){
       
        //if(secArrayExtFr.at(i) != 0){
        //    ROS_WARN("%d",i);
        //    ROS_INFO("%.2f",secArrayExtFr.at(i));  
        //}
    }

   }
}
//Mode 2: This is a ellipse centered at the robot frame, with the long axis along the x axis of the robot frame.

void SecurityMargin::buildElliptic()
{
    geometry_msgs::Point p;
    p.z = 0;
    if (pubMarkers)
    {
        markerIntFr.points.clear();
        markerExtFr.points.clear();
    }
    ROS_ERROR("f1: %f, innerSec %f, extSecDist %f", f1,innerSecDistFront, extSecDistFront);
    for (double i = 0; i < frontLaserArrayMsgLen; i++)
    {
        p.x = f1*innerSecDistFront * cos(i / ((double)frontLaserArrayMsgLen) * 2 * M_PI);
        p.y = innerSecDistFront * sin(i / ((double)frontLaserArrayMsgLen) * 2 * M_PI);
        secArrayFr.push_back(sqrtf(p.x * p.x + p.y * p.y));

        if (pubMarkers &&  ((int)i % 50) == 0 )
            markerIntFr.points.push_back(p);

        p.x *= extSecDistFront / innerSecDistFront;
        p.y *= extSecDistFront / innerSecDistFront;

        secArrayExtFr.push_back(sqrtf(p.x * p.x + p.y * p.y));
        if (pubMarkers &&  ((int)i % 50) == 0)
            markerExtFr.points.push_back(p);
    }
   
}
//Mode 3: Old one used for ARCO laundry robot
void SecurityMargin::buildArrays()
{
    geometry_msgs::Point p;
    p.z = 0;

    for (double i = 0; i < frontLaserArrayMsgLen; i++)
    {
        p.x = innerSecDistFront * cos(i / ((double)frontLaserArrayMsgLen) * M_PI - M_PI_2);
        p.y = f1 * innerSecDistFront * sin(i / ((double)frontLaserArrayMsgLen) * M_PI - M_PI_2);
        secArrayFr.push_back(sqrtf(p.x * p.x + p.y * p.y));
        if ((pubMarkers && ((int)i % 50) == 0 && i >= laserSecurityAngleFront && i <= frontLaserArrayMsgLen - laserSecurityAngleFront) || i == laserSecurityAngleFront || i == frontLaserArrayMsgLen - laserSecurityAngleFront)
            markerIntFr.points.push_back(p);

        p.x *= extSecDistFront / innerSecDistFront;
        p.y *= extSecDistFront / innerSecDistFront;

        secArrayExtFr.push_back(sqrtf(p.x * p.x + p.y * p.y));
        if ((pubMarkers && ((int)i % 50) == 0 && i >= laserSecurityAngleFront && i <= frontLaserArrayMsgLen - laserSecurityAngleFront) || i == laserSecurityAngleFront || i == frontLaserArrayMsgLen - laserSecurityAngleFront)
            markerExtFr.points.push_back(p);
    }

    if (!onlyFront)
    {
        if (pubMarkers)
        {
            markerIntBack.points.clear();
            markerExtBack.points.clear();
        }
        for (double i = 0; i < backLaserArrayMsgLen; i++)
        {
            p.x = innerSecDistBack * cos(i / ((double)backLaserArrayMsgLen) * M_PI - M_PI_2);
            p.y = f2 * innerSecDistBack * sin(i / ((double)backLaserArrayMsgLen) * M_PI - M_PI_2);
            secArrayBack.push_back(sqrtf(p.x * p.x + p.y * p.y));
            if ((pubMarkers && ((int)i % 50) == 0 && i >= laserSecurityAngleBack && i <= backLaserArrayMsgLen - laserSecurityAngleBack) || i == laserSecurityAngleBack || i == backLaserArrayMsgLen - laserSecurityAngleBack)
                markerIntBack.points.push_back(p);

            p.x *= extSecDistBack / innerSecDistBack;
            p.y *= extSecDistBack / innerSecDistBack;
            if ((pubMarkers && ((int)i % 50) == 0 && i >= laserSecurityAngleBack && i <= backLaserArrayMsgLen - laserSecurityAngleBack) || i == laserSecurityAngleBack || i == backLaserArrayMsgLen - laserSecurityAngleBack)
                markerExtBack.points.push_back(p);

            secArrayExtBack.push_back(sqrtf(p.x * p.x + p.y * p.y));
        }
    }
}
void SecurityMargin::publishRvizMarkers()
{
    marker_fr_1_pub.publish(markerIntFr);
    marker_fr_2_pub.publish(markerExtFr);
    if (!onlyFront)
    {
        marker_rr_1_pub.publish(markerIntBack);
        marker_rr_2_pub.publish(markerExtBack);
    }
}
//extPerimeter: boolean to select which ellipse to evaluate, 1 for ext and 0 for inner
bool SecurityMargin::checkObstacles(bool extPerimeter)
{
    if (laser1Got)
    {
        cnt=0;
        cnt2=0;
        
        for (int i = 0; i <  frontLaserArrayMsgLen ; i++)
        {
            //ROS_INFO_THROTTLE(2,"En el for");
            //ROS_INFO_THROTTLE(0.05,"i: %d, int: %.2f, ext: %.2f, laser: %.2f", i,secArrayFr.at(i), secArrayExtFr.at(i), laser1CPtr->ranges.at(i) );
            //It compare the scan element to secArray or secArrayExt depending the value of extPerimeter
            
            if (laser1CPtr->ranges.at(i) < (extPerimeter ? secArrayExtFr.at(i) : secArrayFr.at(i)) && laser1CPtr->ranges.at(i) > laser1CPtr->range_min)
            {

                if(cnt == 0)
                    cnt2=i;

                if(i-cnt2>200)
                    cnt=0;

                cnt++;
                
                if (pubMarkers)
                    SecurityMargin::publishRvizMarkers();
                //ROS_INFO("HEY1");
                if(cnt>25){
                    

                if (!extPerimeter)
                {
                    //ROS_INFO("HEY_EXT");
                    isInsideDangerousArea1 = true;
                    markerIntFr.color = red;
                }
                securityAreaOccup1 = true;
                markerExtFr.color = red;
                red1 = true;
                return red1;
                }
            }
        }

        if (extPerimeter)
        {
            securityAreaOccup1 = false;
            markerExtFr.color = green;
        }

        isInsideDangerousArea1 = false;
        markerIntFr.color = green;

        if (red1)
        {
            red1 = false;
            
            if (pubMarkers)
                SecurityMargin::publishRvizMarkers();
        }
    }

    if (!onlyFront && laser2Got)
    {
        for (int i = laserSecurityAngleBack; i < backLaserArrayMsgLen - laserSecurityAngleBack; i++)
        {
            if (laser2CPtr->ranges.at(i) < (extPerimeter ? secArrayExtBack.at(i) : secArrayBack.at(i)) && laser2CPtr->ranges.at(i) > laser2CPtr->range_min)
            {

                if (!extPerimeter)
                {
                    isInsideDangerousArea2 = true;

                    markerIntBack.color = red;
                }
                securityAreaOccup2 = true;
                markerExtBack.color = red;

                if (!red2 && pubMarkers)
                {
                    SecurityMargin::publishRvizMarkers();
                }
                red2 = true;
                return red2;
            }
        }
        if (extPerimeter)
        {
            securityAreaOccup2 = false;
            markerExtBack.color = green;
        }
        isInsideDangerousArea2 = false;
        markerIntBack.color = green;

        if (red2)
        {
            red2 = false;
            if (pubMarkers)
                SecurityMargin::publishRvizMarkers();
        }
    }

    return false;
}
//This functions will use check obstacles function
// If something inside the dangerous area, returns false
// And it keeps returning false until the object has gone away from the security area(outside the outer ellipse)
bool SecurityMargin::canIMove()
{
    SecurityMargin::refreshParams();

    if(dist2goal.data < 1.5){
        aproximating = true;
    }else{
        aproximating = false;
    }
    
    if(goal_reached.data){
        
        //Aqui se espera el medio metro ese hasta que pone de nuevo aproximating a false

        if (!tr0_catch)
        {
            goingAway = true;
            dlt.x = 0;
            dlt.y = 0;
            tr1.transform.translation.x = 0;
            tr1.transform.translation.y = 0;
            try
            {
                tr0 = tfBuffer->lookupTransform("map", "base_link", ros::Time(0));
                tr0_catch = true;
                
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("No transform %s", ex.what());
            }
        }
        else
        {
            try
            {
                tr1 = tfBuffer->lookupTransform("map", "base_link", ros::Time(0));
                dlt.x = tr1.transform.translation.x - tr0.transform.translation.x;
                dlt.y = tr1.transform.translation.y - tr0.transform.translation.y;
                //ROS_ERROR("Aprox vel: %.2f ; Dist: %.2f", vel.linear.x, sqrtf(dlt.x * dlt.x + dlt.y * dlt.y));
                if (sqrtf(dlt.x * dlt.x + dlt.y * dlt.y) > 1)
                {   
                    aproximating = false;
                    goingAway  =false;
                    goal_reached.data = false;
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("No transform %s", ex.what());
            }
            
                        
        }
    }

    if(!aproximating && !goingAway ){
    
    if (pubMarkers)
        SecurityMargin::publishRvizMarkers();

    if (checkObstacles(0))
    {
        //ROS_WARN("Inside 1");
        if (!stop_msg.data)
        {
            stop_msg.data = true;
            stop_pub.publish(stop_msg);
        }

        return false;
    }

    if(onlyFront)
    {
        if(securityAreaOccup1)
        {
            if (checkObstacles(1))
            {
                //ROS_WARN("Inside 2_1");
                if (!stop_msg.data)
                {
                    stop_msg.data = true;
                    stop_pub.publish(stop_msg);
                }

                return false;
            }
        }

    }
    else if (!SecurityMargin::securityAreaFree())
    {
        if (checkObstacles(1))
        {
            ROS_WARN("Inside 2");
            if (!stop_msg.data)
            {
                stop_msg.data = true;
                stop_pub.publish(stop_msg);
            }

            return false;
        }
    }
    if (stop_msg.data)
    {
        stop_msg.data = false;
        stop_pub.publish(stop_msg);
    }
    }
    return true;
}
bool SecurityMargin::dangerAreaFree()
{
    return isInsideDangerousArea1 || isInsideDangerousArea2;
}
bool SecurityMargin::securityAreaFree()
{
    return !(securityAreaOccup1 || securityAreaOccup2);
}
