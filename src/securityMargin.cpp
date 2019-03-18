
#include <navigator/securityMargin.hpp>



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
    marker_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkers", 1);

    markerInt.header.frame_id = "front_laser_link";
    markerInt.header.stamp = ros::Time();
    markerInt.ns = "debug";
    markerInt.id = 37;
    markerInt.type = visualization_msgs::Marker::SPHERE_LIST;
    markerInt.action = visualization_msgs::Marker::ADD;
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
    markerExt.type = visualization_msgs::Marker::SPHERE_LIST;
    markerExt.action = visualization_msgs::Marker::ADD;
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
    { //TODO: Change 721 to laserArrayMsgLen
        p.x = innerSecDist * cos(i / 721 * M_PI - M_PI_2);
        p.y = f * innerSecDist * sin(i / 721 * M_PI - M_PI_2);
        secArray.push_back(sqrtf(p.x * p.x + p.y * p.y));
        //p.x = x;
        //p.y = y;
        markerInt.points.push_back(p);

        p.x *= extSecDist / innerSecDist;
        p.y *= extSecDist / innerSecDist;
        secArrayExt.push_back(sqrtf(p.x * p.x + p.y * p.y));
        //p.x = x;
        //p.y = y;
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
//This functions will use check obstacles function
// If something inside the dangerous area, returns false
// And it keeps returning false until the object has gone away from the security area(outside the outer ellipse)
bool SecurityMargin::canIMove(sensor_msgs::LaserScan *scan){

    if(checkObstacles(0,scan))
        return false;
    
    //If we arrived here, it means that there is no obstacle inside dangerous area 
    //But the obstacle could be still between the two margins, so we check if it's inside the exterior margins
    if(!checkObstacles(1,scan))
        return true;
    
    //Finally by default it returns false
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
