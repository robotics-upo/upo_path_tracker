
#include <navigator/securityMargin.hpp>

SecurityMargin::SecurityMargin(ros::NodeHandle *n)
{
    SecurityMargin::setParams(n);
    SecurityMargin::buildArrays();
}
void SecurityMargin::setParams(ros::NodeHandle *n)
{
    //Parse parameters
    nh = n;
    marker_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkers", 1);
    
    float laserSecurityAngle_;
    
    nh->param("/security_margin/only_front", onlyFront, (bool) 1);
    nh->param("/security_margin/laser_range_msg_length", laserArrayMsgLen, (int) 721);
    nh->param("/security_margin/f_relationship",f,(float)1.6);
    nh->param("/security_margin/inner_radius", innerSecDist, (float) 0.2);
    nh->param("/security_margin/outer_radius",extSecDist, (float) 0.6);
    nh->param("/security_margin/laser_security_angle/",laserSecurityAngle_, (float)15);

    laserSecurityAngle = ceil(laserSecurityAngle_ * (laserArrayMsgLen / 180));

    //Initialize inner margin markers
    markerInt.header.frame_id = "front_laser_link";
    markerInt.header.stamp = ros::Time();
    markerInt.ns = "debug";
    markerInt.id = 37;
    markerInt.type = visualization_msgs::Marker::LINE_STRIP;
    markerInt.action = visualization_msgs::Marker::ADD;
    markerInt.scale.x = 0.1;
    markerInt.color.a = 1.0;
    markerInt.color.r = 0.0;
    markerInt.color.g = 0.0;
    markerInt.color.b = 1.0;

    //Initialize exterior margin markers
    markerExt.header.frame_id = "front_laser_link";
    markerExt.header.stamp = ros::Time();
    markerExt.ns = "debug";
    markerExt.id = 12;
    markerExt.type = visualization_msgs::Marker::LINE_STRIP;
    markerExt.action = visualization_msgs::Marker::ADD;
    markerExt.scale.x = 0.1;
    markerExt.color.a = 1.0;
    markerExt.color.r = 0.0;
    markerExt.color.g = 0.0;
    markerExt.color.b = 1.0;

    //Control flags
    isInsideDangerousArea = false;
    securityAreaOccup = false;
    paramsConfigured = true;
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
        p.x = innerSecDist * cos(i / ((double)laserArrayMsgLen)* M_PI - M_PI_2);
        p.y = f * innerSecDist * sin(i / ((double)laserArrayMsgLen) * M_PI - M_PI_2);
        secArray.push_back(sqrtf(p.x * p.x + p.y * p.y));
        markerInt.points.push_back(p);

        p.x *= extSecDist / innerSecDist;
        p.y *= extSecDist / innerSecDist;
        secArrayExt.push_back(sqrtf(p.x * p.x + p.y * p.y));
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
        //It compare the scan element to secArray or secArrayExt depending the value of whichOne
        if (scan->ranges.at(i) < (whichOne ? secArrayExt.at(i) : secArray.at(i)) && scan->ranges.at(i) > scan->range_min)
        {

            if (!whichOne)
            {
                isInsideDangerousArea = true;

                markerInt.color.r = 1.0;
                markerInt.color.b = 0;
                markerInt.color.g = 0;
            }
            securityAreaOccup = true;
            markerExt.color.r = 1.0;
            markerExt.color.b = 0;
            markerExt.color.g = 0;
            return true;
        }
    }
    //If we ara evaluating the exterior ellipses, and it didn't find nothing inside this ellipse, it changes the flags values
    if (whichOne)
    {
        securityAreaOccup = false;
        markerExt.color.r = 0;
        markerExt.color.b = 1.0;
        markerExt.color.g = 0;
    }
    isInsideDangerousArea = false;
    markerInt.color.r = 0;
    markerInt.color.b = 1.0;
    markerInt.color.g = 0;

    return false;
}
//This functions will use check obstacles function
// If something inside the dangerous area, returns false
// And it keeps returning false until the object has gone away from the security area(outside the outer ellipse)

bool SecurityMargin::canIMove(sensor_msgs::LaserScan *scan)
{
    if (checkObstacles(0, scan))
        return false;

    if (securityAreaOccup)
    {
        if (checkObstacles(1, scan))
        {
            return false;
        }
    }
    return true;
}
bool SecurityMargin::dangerAreaFree()
{
    return isInsideDangerousArea;
}

bool SecurityMargin::securityAreaFree()
{
    return !securityAreaOccup;
}
