
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

    nh->param("security_margin/only_front", onlyFront, (bool)0);
    ROS_ERROR("ONLYF: %d", onlyFront);
    nh->param("security_margin/laser_range_msg_length", laserArrayMsgLen, (int)721); //721 for only front with hokuyo
    nh->param("security_margin/f_relationship", f, (float)1.6);
    nh->param("security_margin/inner_radius", innerSecDist, (float)0.6);
    nh->param("security_margin/outer_radius", extSecDist, (float)1);
    //This parameter will only be used in only front mode
    nh->param("security_margin/laser_security_angle/", laserSecurityAngle_, (float)15);

    laserSecurityAngle = ceil(laserSecurityAngle_ * (laserArrayMsgLen / 180));

    //Initialize inner margin markers
    markerIntFr.header.frame_id = "front_laser_link";
    markerIntFr.header.stamp = ros::Time();
    markerIntFr.ns = "debug";
    markerIntFr.id = 37;
    markerIntFr.type = visualization_msgs::Marker::LINE_STRIP;
    markerIntFr.action = visualization_msgs::Marker::ADD;
    markerIntFr.scale.x = 0.025;
    markerIntFr.color.a = 1.0;
    markerIntFr.color.r = 0.0;
    markerIntFr.color.g = 1.0;
    markerIntFr.color.b = 0.0;

    //Initialize inner margin markers
    markerIntBack.header.frame_id = "back_laser_link";
    markerIntBack.header.stamp = ros::Time();
    markerIntBack.ns = "debug";
    markerIntBack.id = 39;
    markerIntBack.type = visualization_msgs::Marker::LINE_STRIP;
    markerIntBack.action = visualization_msgs::Marker::ADD;
    markerIntBack.scale.x = 0.025;
    markerIntBack.color.a = 1.0;
    markerIntBack.color.r = 0.0;
    markerIntBack.color.g = 1.0;
    markerIntBack.color.b = 0.0;

    //Initialize exterior margin markers
    markerExtFr.header.frame_id = "front_laser_link";
    markerExtFr.header.stamp = ros::Time();
    markerExtFr.ns = "debug";
    markerExtFr.id = 12;
    markerExtFr.type = visualization_msgs::Marker::LINE_STRIP;
    markerExtFr.action = visualization_msgs::Marker::ADD;
    markerExtFr.scale.x = 0.025;
    markerExtFr.color.a = 1.0;
    markerExtFr.color.r = 0.0;
    markerExtFr.color.g = 1.0;
    markerExtFr.color.b = 0.0;

    markerExtBack.header.frame_id = "back_laser_link";
    markerExtBack.header.stamp = ros::Time();
    markerExtBack.ns = "debug";
    markerExtBack.id = 13;
    markerExtBack.type = visualization_msgs::Marker::LINE_STRIP;
    markerExtBack.action = visualization_msgs::Marker::ADD;
    markerExtBack.scale.x = 0.025;
    markerExtBack.color.a = 1.0;
    markerExtBack.color.r = 0.0;
    markerExtBack.color.g = 1.0;
    markerExtBack.color.b = 0.0;

    //Control flags
    isInsideDangerousArea1 = false;
    securityAreaOccup1 = false;
    isInsideDangerousArea2 = false;
    securityAreaOccup2 = false;
    paramsConfigured = true;
}

void SecurityMargin::buildArrays()
{
    double x, y;
    geometry_msgs::Point p;
    p.z = 0;
    markerIntFr.points.clear();
    markerExtFr.points.clear();

    markerIntBack.points.clear();
    markerExtBack.points.clear();
    ROS_INFO("Before build");
    for (double i = 0; i < laserArrayMsgLen; i++)
    {
        p.x = innerSecDist * cos(i / ((double)laserArrayMsgLen) * M_PI - M_PI_2);
        p.y = f * innerSecDist * sin(i / ((double)laserArrayMsgLen) * M_PI - M_PI_2);
        secArrayFr.push_back(sqrtf(p.x * p.x + p.y * p.y));

        markerIntFr.points.push_back(p);
        if (!onlyFront)
        {
            secArrayBack.push_back(sqrtf(p.x * p.x + p.y * p.y));
            markerIntBack.points.push_back(p);
        }
        p.x *= extSecDist / innerSecDist;
        p.y *= extSecDist / innerSecDist;

        secArrayExtFr.push_back(sqrtf(p.x * p.x + p.y * p.y));
        markerExtFr.points.push_back(p);
        if (!onlyFront)
        {
            markerExtBack.points.push_back(p);
            secArrayExtBack.push_back(sqrtf(p.x * p.x + p.y * p.y));
        }
    }
}

void SecurityMargin::publishRvizMarkers()
{
    marker_pub.publish(markerIntFr);
    marker_pub.publish(markerExtFr);

    marker_pub.publish(markerIntBack);
    marker_pub.publish(markerExtBack);
}
//whichOne: boolean to select which ellipse to evaluate, 1 for ext and 0 for inner
bool SecurityMargin::checkObstacles(bool whichOne, sensor_msgs::LaserScan *scan1, sensor_msgs::LaserScan *scan2)
{
    for (int i = laserSecurityAngle; i < laserArrayMsgLen - laserSecurityAngle; i++)
    {
        //ROS_INFO("hEY, %d",i);
        //It compare the scan element to secArray or secArrayExt depending the value of whichOne
        if (scan1->ranges.at(i) < (whichOne ? secArrayExtFr.at(i) : secArrayFr.at(i)) && scan1->ranges.at(i) > scan1->range_min)
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
        }
    }
    //If we ara evaluating the exterior ellipses, and it didn't find nothing inside this ellipse, it changes the flags values
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

    if (!onlyFront)
    {
        //ROS_INFO("HOLAAA");
        for (int i = laserSecurityAngle; i < laserArrayMsgLen - laserSecurityAngle; i++)
        {
            // ROS_INFO("hEY2 %d", i);
            //It compare the scan element to secArray or secArrayExt depending the value of whichOne
            if (scan2->ranges.at(i) < (whichOne ? secArrayExtBack.at(i) : secArrayBack.at(i)) && scan2->ranges.at(i) > scan2->range_min)
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
                return true;
            }
        }
        //If we ara evaluating the exterior ellipses, and it didn't find nothing inside this ellipse, it changes the flags values
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
    }

    return false;
}
//This functions will use check obstacles function
// If something inside the dangerous area, returns false
// And it keeps returning false until the object has gone away from the security area(outside the outer ellipse)

bool SecurityMargin::canIMove(sensor_msgs::LaserScan *scan1, sensor_msgs::LaserScan *scan2)
{
    //ROS_INFO("CHECKIN INNER");
    if (checkObstacles(0, scan1, scan2))
        return false;

    if (securityAreaOccup1 || securityAreaOccup2)
    {
        if (checkObstacles(1, scan1, scan2))
        {
            return false;
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
