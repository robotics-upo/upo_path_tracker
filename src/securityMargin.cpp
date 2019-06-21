
#include <navigator/securityMargin.hpp>

SecurityMargin::SecurityMargin(ros::NodeHandle *n)
{

    SecurityMargin::setParams(n);

   
    SecurityMargin::buildArraysSquare2(&secArrayFr, &markerIntFr,0);
    SecurityMargin::buildArraysSquare2(&secArrayExtFr, &markerExtFr,1);
    
    if (pubMarkers)
        SecurityMargin::publishRvizMarkers();
}
void SecurityMargin::laser1Callback(const sensor_msgs::LaserScanConstPtr &scan) //Front
{
    laser1CPtr = scan;

    l1_angle_i = scan->angle_increment;
    l1_angle_min = scan->angle_min;
    l1_lenght = scan->ranges.size();
    laser1Got = true;
    if (laser2Got)
        lasersGot = true;
}
void SecurityMargin::laser2Callback(const sensor_msgs::LaserScanConstPtr &scan) //Rear
{
    laser2CPtr = scan;
    laser2Got = true;
    l2_angle_i = scan->angle_increment;
    l2_angle_min = scan->angle_min;
    l2_lenght = scan->ranges.size();
    if (laser1Got)
        lasersGot = true;
}
void SecurityMargin::refreshParams()
{

    float laserSecurityAngleBack_, laserSecurityAngleFront_;

    ros::param::get("nav_node/f_relationship_front", f1);
    ros::param::get("nav_node/f_relationship_back", f2);
    ros::param::get("nav_node/inner_radius_front", innerSecDistFront);
    ros::param::get("nav_node/outer_radius_front", extSecDistFront);
    ros::param::get("nav_node/inner_radius_back", innerSecDistBack);
    ros::param::get("nav_node/outer_radius_back", extSecDistBack);
    ros::param::get("nav_node/laser_security_angle_front", laserSecurityAngleFront_);
    ros::param::get("nav_node/laser_security_angle_back", laserSecurityAngleBack_);
    ros::param::get("nav_node/only_front", onlyFront);
    ros::param::get("nav_node/hard_stop_enabled", hard_stop_enabled);
    
    laserSecurityAngleFront = ceil(laserSecurityAngleFront_ * (frontLaserArrayMsgLen / 180));
    laserSecurityAngleBack = ceil(laserSecurityAngleBack_ * (backLaserArrayMsgLen / 180));
}
void SecurityMargin::setParams(ros::NodeHandle *n)
{
    nh = n;
    //Markers publishers
    marker_fr_1_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersFr1", 0);
    marker_fr_2_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersFr2", 0);
    marker_rr_1_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersRr1", 0);
    marker_rr_2_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersRr2", 0);
    stop = nh->advertise<std_msgs::Bool>("/security_stop", 0);

    float laserSecurityAngleBack_, laserSecurityAngleFront_;

    nh->param("nav_node/only_front", onlyFront, (bool)0);
    nh->param("nav_node/front_laser_range_msg_length", frontLaserArrayMsgLen, (int)HOKUYO); //721 for hokuyo lasers by default
    nh->param("nav_node/back_laser_range_msg_length", backLaserArrayMsgLen, (int)HOKUYO);   //721 for hokuyo lasers by default
    nh->param("nav_node/f_relationship_front", f1, (float)1.6);
    nh->param("nav_node/f_relationship_back", f2, (float)1.6);
    nh->param("nav_node/inner_radius_front", innerSecDistFront, (float)0.6);
    nh->param("nav_node/outer_radius_front", extSecDistFront, (float)1);
    nh->param("nav_node/inner_radius_back", innerSecDistBack, (float)0.6);
    nh->param("nav_node/outer_radius_back", extSecDistBack, (float)1);
    nh->param("nav_node/publish_markers", pubMarkers, (bool)1);
    nh->param("nav_node/laser_security_angle_front", laserSecurityAngleFront_, (float)15);
    nh->param("nav_node/laser_security_angle_back", laserSecurityAngleBack_, (float)15);
    nh->param("nav_node/hard_stop_enabled", hard_stop_enabled, (bool)1);
    nh->param("nav_node/min_angle_laser_1", l1_angle_min, (double)-1.57);
    nh->param("nav_node/min_angle_laser_2", l2_angle_min, (double)-1.57);
    nh->param("nav_node/step_laser",l1_angle_i, (double)0.004);
    nh->param("nav_node/square_security_area/h1",h1,(float)0.2);
    nh->param("nav_node/square_security_area/width",w,(float)0.2);
    nh->param("nav_node/square_security_area/h2",h2,(float)0.2 );
    nh->param("nav_node/square_security_area/delta_d",delta_d,(float)0.15 );
    nh->param("nav_node/square_security_area/margin_x",margin_x,(float)0.5 );
    nh->param("nav_node/square_security_area/margin_y",margin_y,(float)0.5 );
    l2_angle_i = l1_angle_i;
    laserSecurityAngleFront = ceil(laserSecurityAngleFront_ * (frontLaserArrayMsgLen / 180));
    laserSecurityAngleBack = ceil(laserSecurityAngleBack_ * (backLaserArrayMsgLen / 180));

    //Control flags
    isInsideDangerousArea1 = false;
    securityAreaOccup1 = false;
    
    isInsideDangerousArea2 = false;
    securityAreaOccup2 = false;

    paramsConfigured = true;

    red1 = false;
    red2 = false;

    laser1Got = false;
    laser2Got = false;
    lasersGot = false;

    if (pubMarkers)
    {
        markerIntFr.header.frame_id = "base_link";
        markerIntFr.header.stamp = ros::Time();
        markerIntFr.ns = "debug";
        markerIntFr.id = 37;
        markerIntFr.type = visualization_msgs::Marker::LINE_STRIP;
        markerIntFr.action = visualization_msgs::Marker::ADD;
        markerExtBack.lifetime = ros::DURATION_MAX;
        markerIntFr.scale.x = 0.025;
        markerIntFr.color.a = 1.0;
        markerIntFr.color.r = 0.0;
        markerIntFr.color.g = 1.0;
        markerIntFr.color.b = 0.0;

        markerIntBack.header.frame_id = "back_laser_link";
        markerIntBack.header.stamp = ros::Time();
        markerIntBack.ns = "debug";
        markerIntBack.id = 38;
        markerIntBack.type = visualization_msgs::Marker::LINE_STRIP;
        markerIntBack.action = visualization_msgs::Marker::ADD;
        markerExtBack.lifetime = ros::DURATION_MAX;
        markerIntBack.scale.x = 0.025;
        markerIntBack.color.a = 1.0;
        markerIntBack.color.r = 0.0;
        markerIntBack.color.g = 1.0;
        markerIntBack.color.b = 0.0;

        markerExtFr.header.frame_id = "base_link";
        markerExtFr.header.stamp = ros::Time();
        markerExtFr.ns = "debug";
        markerExtFr.id = 39;
        markerExtFr.type = visualization_msgs::Marker::LINE_STRIP;
        markerExtFr.action = visualization_msgs::Marker::ADD;
        markerExtBack.lifetime = ros::DURATION_MAX;
        markerExtFr.scale.x = 0.025;
        markerExtFr.color.a = 1.0;
        markerExtFr.color.r = 0.0;
        markerExtFr.color.g = 1.0;
        markerExtFr.color.b = 0.0;

        markerExtBack.header.frame_id = "base_link";
        markerExtBack.header.stamp = ros::Time();
        markerExtBack.ns = "debug";
        markerExtBack.id = 40;
        markerExtBack.type = visualization_msgs::Marker::LINE_STRIP;
        markerExtBack.action = visualization_msgs::Marker::ADD;
        markerExtBack.lifetime = ros::DURATION_MAX;
        markerExtBack.scale.x = 0.025;
        markerExtBack.color.a = 1.0;
        markerExtBack.color.r = 0.0;
        markerExtBack.color.g = 1.0;
        markerExtBack.color.b = 0.0;
    }
}
//Si calculo los puntos por pares
/*

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
void SecurityMargin::buildArraysSquare2(vector<float> *array, RVizMarker *marker, bool ext){
    pair<float, float> p1,p2,p3;
    vector<float> a1,a2;
    
    p1.first = ext?margin_x+delta_d:margin_x;
    p1.second = 0;
   
    p2.first = ext?margin_x+delta_d:margin_x;
    p2.second = ext?margin_y+delta_d:margin_y;

    p3.first = 0;
    p3.second = ext?margin_y+delta_d:margin_y;

    float L1,L2,L;
    float incr1,incr2;
    int n1,n2;
    
    L1 = p1.first;
    L2 = p3.second;
    L = 4*L1+4*L2;

    n1 = floor(frontLaserArrayMsgLen* L1/L);
    n2 = floor(frontLaserArrayMsgLen* L2/L);
    //ROS_WARN("N1: %d \t N2: %d",n1,n2);
    incr1 = L1/n1;
    incr2 = L2/n2;
    
    
    //Sec de push_back: L1,L2,L2,L1,L1,L2,L2,L1
    for(int i = 0; i < n1; i++){
        a1.push_back(sqrtf( p1.first*p1.first +i*incr1*i*incr1));
    
    }
    for(int i = 0; i < n2; i++){   
        a2.push_back(sqrtf( p2.second*p2.second + (p2.first-i*incr2)*(p2.first-i*incr2)));
    }

    for(int i = 0; i< n1; i++){
        array->push_back(a1.at(i));
        //ROS_WARN("%.2f", a1.at(i));
    }
    for(int i = 0; i < n2;i++){
        array->push_back(a2.at(i));
    }
    for(int i = n2; i > 0; i--){
        array->push_back(a2.at(i-1));
    }
    for(int i = n1; i > 0; i--){
        array->push_back(a1.at(i-1));
    }
    for(int i = 0; i < n1; i++){
        array->push_back(a1.at(i));
    }
    
   
    for(int i = 0; i < n2; i++){
        array->push_back(a2.at(i));
    }
    for(int i = n2; i > 0; i--){
        array->push_back(a2.at(i-1));
    }
    for(int i = n1; i > 0; i--){
        array->push_back(a1.at(i-1));
         //ROS_WARN("%.2f", a1.at(i-1));
    }
    while(array->size() < frontLaserArrayMsgLen){
        array->push_back(0); 
       
    }
    marker->points.clear();
    geometry_msgs::Point p;
    p.x = p2.first;
    p.y = p2.second;

    marker->points.push_back(p);
    p.y*=-1;
    marker->points.push_back(p);
    p.x*=-1;
    marker->points.push_back(p);
    p.y*=-1;
    marker->points.push_back(p);
    p.x*=-1;
    marker->points.push_back(p);
  
}
void SecurityMargin::buildArrays()
{
    double x, y;
    geometry_msgs::Point p;
    p.z = 0;
    if (pubMarkers)
    {
        markerIntFr.points.clear();
        markerExtFr.points.clear();
    }
    int it = 0;

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
//whichOne: boolean to select which ellipse to evaluate, 1 for ext and 0 for inner
bool SecurityMargin::checkObstacles(bool whichOne)
{
    if (lasersGot)
    {

        for(int i = 0; i < frontLaserArrayMsgLen; i++)//for (int i = laserSecurityAngleFront; i < frontLaserArrayMsgLen - laserSecurityAngleFront; i++)
        {
            //It compare the scan element to secArray or secArrayExt depending the value of whichOne
            if (laser1CPtr->ranges.at(i) < (whichOne ? secArrayExtFr.at(i) : secArrayFr.at(i)) && laser1CPtr->ranges.at(i) > laser1CPtr->range_min)
            {

                if (!whichOne)
                {
                    isInsideDangerousArea1 = true;

                    markerIntFr.color.r = 1.0;
                    markerIntFr.color.b = 0;
                    markerIntFr.color.g = 0;
                }
                securityAreaOccup1 = true;
                markerExtFr.color.r = 1.0;
                markerExtFr.color.b = 0;
                markerExtFr.color.g = 0;
                return true;

                if (!red1 && pubMarkers)
                    SecurityMargin::publishRvizMarkers();

                red1 = true;
            }
        }

        if (whichOne)
        {
            securityAreaOccup1 = false;
            markerExtFr.color.r = 0;
            markerExtFr.color.b = 0;
            markerExtFr.color.g = 1.0;
        }

        isInsideDangerousArea1 = false;
        markerIntFr.color.r = 0;
        markerIntFr.color.b = 0;
        markerIntFr.color.g = 1.0;
        if (red1)
        {
            red1 = false;
            if (pubMarkers)
                SecurityMargin::publishRvizMarkers();
        }
        if (!onlyFront)
        {
            for (int i = laserSecurityAngleBack; i < backLaserArrayMsgLen - laserSecurityAngleBack; i++)
            {
                if (laser2CPtr->ranges.at(i) < (whichOne ? secArrayExtBack.at(i) : secArrayBack.at(i)) && laser2CPtr->ranges.at(i) > laser2CPtr->range_min)
                {

                    if (!whichOne)
                    {
                        isInsideDangerousArea2 = true;

                        markerIntBack.color.r = 1.0;
                        markerIntBack.color.b = 0;
                        markerIntBack.color.g = 0;
                    }
                    securityAreaOccup2 = true;
                    markerExtBack.color.r = 1.0;
                    markerExtBack.color.b = 0;
                    markerExtBack.color.g = 0;

                    if (!red2 && pubMarkers)
                    {
                        SecurityMargin::publishRvizMarkers();
                    }
                    red2 = true;
                    return true;
                }
            }
            if (whichOne)
            {
                securityAreaOccup2 = false;
                markerExtBack.color.r = 0;
                markerExtBack.color.b = 0;
                markerExtBack.color.g = 1.0;
            }
            isInsideDangerousArea2 = false;
            markerIntBack.color.r = 0;
            markerIntBack.color.b = 0;
            markerIntBack.color.g = 1.0;
            if (red2)
            {
                red2 = false;
                if (pubMarkers)
                    SecurityMargin::publishRvizMarkers();
            }
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

    if (pubMarkers)
        SecurityMargin::publishRvizMarkers();

    if (checkObstacles(0))
    {
        if (!stop_msg.data)
        {
            stop_msg.data = true;
            stop.publish(stop_msg);
        }

        return false;
    }

    if (!SecurityMargin::securityAreaFree())
    {
        if (checkObstacles(1))
        {
            if (!stop_msg.data)
            {
                stop_msg.data = true;
                stop.publish(stop_msg);
            }

            return false;
        }
    }
    if (stop_msg.data)
    {
        stop_msg.data = false;
        stop.publish(stop_msg);
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
