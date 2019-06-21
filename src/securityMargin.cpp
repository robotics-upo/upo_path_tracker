
#include <navigator/securityMargin.hpp>

SecurityMargin::SecurityMargin(ros::NodeHandle *n)
{
    SecurityMargin::setParams(n);

    switch (secMode)
    {
    case 0:
        SecurityMargin::buildArrays();
        break;
    case 1:
        SecurityMargin::buildArraysSquare2(&secArrayFr, &markerIntFr, 0);
        SecurityMargin::buildArraysSquare2(&secArrayExtFr, &markerExtFr, 1);
        break;
    case 2:
        SecurityMargin::buildElliptic();
        break;
    default:
        break;
    }

    if (pubMarkers)
        SecurityMargin::publishRvizMarkers();
}
 void SecurityMargin::laser1Callback(const sensor_msgs::LaserScanConstPtr &scan) //Front
{

    if (!laser1Got)
    {
        laser1CPtr = scan;
        frontLaserArrayMsgLen = scan->ranges.size();
        laser1Got = true;
        if (secMode = 0)
        {
            float laserSecurityAngleFront_;
            nh->param("nav_node/security_margin/laser_security_angle_front", laserSecurityAngleFront_, (float)15);
            laserSecurityAngleFront = ceil(laserSecurityAngleFront_ * (frontLaserArrayMsgLen / 180));
        }
    }
    if (laser2Got && !lasersGot)
        lasersGot = true;
}
void SecurityMargin::laser2Callback(const sensor_msgs::LaserScanConstPtr &scan) //Rear
{

    if (!laser2Got)
    {
        laser2CPtr = scan;
        backLaserArrayMsgLen = scan->ranges.size();
        laser2Got = true;
        if (secMode == 0)
        {
            float laserSecurityAngleBack_;
            nh->param("nav_node/security_margin/laser_security_angle_back", laserSecurityAngleBack_, (float)15);
            laserSecurityAngleBack = ceil(laserSecurityAngleBack_ * (backLaserArrayMsgLen / 180));
        }
    }

    if (laser1Got && !lasersGot)
        lasersGot = true;
}
void SecurityMargin::refreshParams()
{
    float laserSecurityAngleBack_, laserSecurityAngleFront_;

    ros::param::get("nav_node/security_margin/f_front", f1);
    ros::param::get("nav_node/security_margin/f_back", f2);
    ros::param::get("nav_node/security_margin/inner_radius_front", innerSecDistFront);
    ros::param::get("nav_node/security_margin/outer_radius_front", extSecDistFront);
    ros::param::get("nav_node/security_margin/inner_radius_back", innerSecDistBack);
    ros::param::get("nav_node/security_margin/outer_radius_back", extSecDistBack);

    laserSecurityAngleFront = ceil(laserSecurityAngleFront_ * (frontLaserArrayMsgLen / 180));
    laserSecurityAngleBack = ceil(laserSecurityAngleBack_ * (backLaserArrayMsgLen / 180));
}
void SecurityMargin::setParams(ros::NodeHandle *n)
{
    nh = n;
    //Markers publishers

    nh->param("nav_node/security_margin/security_mode", secMode, (int)1);
    nh->param("nav_node/security_margin/publish_markers", pubMarkers, (bool)1);
    nh->param("nav_node/security_margin/front_laser_link_frame_id", front_laser_link_frame, (string) "front_laser_link");
    string stop_topic;
    nh->param("nav_node/security_margin/security_stop_topic", stop_topic, (string) "/security_stop");
    stop_pub = nh->advertise<std_msgs::Bool>(stop_topic, 0);

    switch (secMode)
    {

    case 0:
        nh->param("nav_node/security_margin/f_front", f1, (float)1.6);
        nh->param("nav_node/security_margin/f_back", f2, (float)1.6);
        nh->param("nav_node/security_margin/inner_radius_front", innerSecDistFront, (float)0.6);
        nh->param("nav_node/security_margin/outer_radius_front", extSecDistFront, (float)1);
        nh->param("nav_node/security_margin/inner_radius_back", innerSecDistBack, (float)0.6);
        nh->param("nav_node/security_margin/outer_radius_back", extSecDistBack, (float)1);
        nh->param("nav_node/security_margin/only_front", onlyFront, (bool)0);
        nh->param("nav_node/security_margin/back_laser_link_frame_id", back_laser_link_frame, (string) "back_laser_link");
        nh->param("nav_node/security_margin/laser_front_topic", laser1_topic, (string)"/scanFront");
        nh->param("nav_node/security_margin/laser_rear_topic", laser2_topic, (string)"/scanBack");
        break;
    case 1:
        onlyFront = true;
        nh->param("nav_node/security_margin/robot_frame_id", base_link_frame, (string) "base_link");
        nh->param("nav_node/security_margin/square_security_area/delta_d", delta_d, (float)0.15);
        nh->param("nav_node/security_margin/square_security_area/margin_x", margin_x, (float)0.5);
        nh->param("nav_node/security_margin/square_security_area/margin_y", margin_y, (float)0.5);
        nh->param("nav_node/security_margin/full_laser_topic", laser1_topic, (string)"/scanMulti");
        break;
    case 2:
        onlyFront = true;
        nh->param("nav_node/security_margin/f", f1, (float)1.6);
        nh->param("nav_node/security_margin/inner_radius", innerSecDistFront, (float)0.6);
        nh->param("nav_node/security_margin/outer_radius", extSecDistFront, (float)1);
        nh->param("nav_node/security_margin/full_laser_topic", laser1_topic, (string)"/scanMulti");
        break;
    default:
        ROS_ERROR("Security Margin: Wrong mode, %d doesn't exist", secMode);
        break;
    }
    
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

    red.a = 1;
    red.b = 0;
    red.g = 0;
    red.r = 1;

    green.a = 1;
    green.g = 1;
    green.b = 0;
    green.r = 0;

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
void SecurityMargin::buildArraysSquare2(vector<float> *array, RVizMarker *marker, bool ext)
{
    pair<float, float> p1, p2, p3;
    vector<float> a1, a2;

    p1.first = ext ? margin_x + delta_d : margin_x;
    p1.second = 0;

    p2.first = ext ? margin_x + delta_d : margin_x;
    p2.second = ext ? margin_y + delta_d : margin_y;

    p3.first = 0;
    p3.second = ext ? margin_y + delta_d : margin_y;

    float L1, L2, L;
    float incr1, incr2;
    int n1, n2;

    L1 = p1.first;
    L2 = p3.second;
    L = 4 * L1 + 4 * L2;

    n1 = floor(frontLaserArrayMsgLen * L1 / L);
    n2 = floor(frontLaserArrayMsgLen * L2 / L);
    //ROS_WARN("N1: %d \t N2: %d",n1,n2);
    incr1 = L1 / n1;
    incr2 = L2 / n2;

    //Sec de push_back: L1,L2,L2,L1,L1,L2,L2,L1
    for (int i = 0; i < n1; i++)
    {
        a1.push_back(sqrtf(p1.first * p1.first + i * incr1 * i * incr1));
    }
    for (int i = 0; i < n2; i++)
    {
        a2.push_back(sqrtf(p2.second * p2.second + (p2.first - i * incr2) * (p2.first - i * incr2)));
    }

    for (int i = 0; i < n1; i++)
    {
        array->push_back(a1.at(i));
        //ROS_WARN("%.2f", a1.at(i));
    }
    for (int i = 0; i < n2; i++)
    {
        array->push_back(a2.at(i));
    }
    for (int i = n2; i > 0; i--)
    {
        array->push_back(a2.at(i - 1));
    }
    for (int i = n1; i > 0; i--)
    {
        array->push_back(a1.at(i - 1));
    }
    for (int i = 0; i < n1; i++)
    {
        array->push_back(a1.at(i));
    }

    for (int i = 0; i < n2; i++)
    {
        array->push_back(a2.at(i));
    }
    for (int i = n2; i > 0; i--)
    {
        array->push_back(a2.at(i - 1));
    }
    for (int i = n1; i > 0; i--)
    {
        array->push_back(a1.at(i - 1));
    }
    while (array->size() < frontLaserArrayMsgLen)
    {
        array->push_back(0);
    }
    marker->points.clear();
    geometry_msgs::Point p;
    p.x = p2.first;
    p.y = p2.second;

    marker->points.push_back(p);
    p.y *= -1;
    marker->points.push_back(p);
    p.x *= -1;
    marker->points.push_back(p);
    p.y *= -1;
    marker->points.push_back(p);
    p.x *= -1;
    marker->points.push_back(p);
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
    for (double i = 0; i < frontLaserArrayMsgLen; i++)
    {
        p.x = innerSecDistFront * cos(i / ((double)frontLaserArrayMsgLen) * 2 * M_PI);
        p.y = f1 * innerSecDistFront * sin(i / ((double)frontLaserArrayMsgLen) * 2 * M_PI);
        secArrayFr.push_back(sqrtf(p.x * p.x + p.y * p.y));

        if ((pubMarkers &&
             ((int)i % 50) == 0 && i >= laserSecurityAngleFront && i <= frontLaserArrayMsgLen - laserSecurityAngleFront) ||
            i == laserSecurityAngleFront || i == frontLaserArrayMsgLen - laserSecurityAngleFront)
            markerIntFr.points.push_back(p);

        p.x *= extSecDistFront / innerSecDistFront;
        p.y *= extSecDistFront / innerSecDistFront;

        secArrayExtFr.push_back(sqrtf(p.x * p.x + p.y * p.y));
        if ((pubMarkers &&
             ((int)i % 50) == 0 && i >= laserSecurityAngleFront && i <= frontLaserArrayMsgLen - laserSecurityAngleFront) ||
            i == laserSecurityAngleFront || i == frontLaserArrayMsgLen - laserSecurityAngleFront)
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
        int i;
        for ((secMode == 0 ? i = laserSecurityAngleFront : i = 0); i < (secMode == 0 ? frontLaserArrayMsgLen - laserSecurityAngleFront : frontLaserArrayMsgLen); i++)
        {
            //It compare the scan element to secArray or secArrayExt depending the value of extPerimeter
            if (laser1CPtr->ranges.at(i) < (extPerimeter ? secArrayExtFr.at(i) : secArrayFr.at(i)) && laser1CPtr->ranges.at(i) > laser1CPtr->range_min)
            {

                securityAreaOccup1 = true;
                markerExtFr.color = red;

                if (pubMarkers)
                    SecurityMargin::publishRvizMarkers();

                if (!extPerimeter)
                {
                    isInsideDangerousArea1 = true;
                    markerIntFr.color = red;
                }
                red1 = true;
                return red1;
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

    if (pubMarkers)
        SecurityMargin::publishRvizMarkers();

    if (checkObstacles(0))
    {
        if (!stop_msg.data)
        {
            stop_msg.data = true;
            stop_pub.publish(stop_msg);
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
