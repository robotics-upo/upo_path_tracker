
#include <arco_path_tracker/SecurityMargin.hpp>

namespace Upo
{
namespace Navigation
{
SecurityMargin::SecurityMargin()
{
  setParams();
}

void SecurityMargin::laser1Callback(const sensor_msgs::LaserScanConstPtr &scan)  // Front
{
  // ROS_INFO("Testing laser 1: %.2f", scan->ranges.at(40));
  laserCPtr = scan;
  if (!laserGot)
  {
    laserMsgLen = scan->ranges.size();
    laserGot = true;
    buildElliptic();
  }
}
void SecurityMargin::setMode(int mode)
{
  if (mode)
  {
    status = 4;
  }
  else
  {
    status = 0;
  }
}
void SecurityMargin::setParams()
{
  nh.reset(new ros::NodeHandle("~"));

  nh->param("robot_base_frame", base_link_frame, (string) "base_link");
  nh->param("f", f, (float)1);
  nh->param("inner_radius", innerSecDistFront, (float)0.3);
  nh->param("outer_radius", extSecDistFront, (float)0.45);

  stop_pub = nh->advertise<std_msgs::Bool>("security_stop", 1);
  marker_int_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersInt", 2);
  marker_ext_pub = nh->advertise<visualization_msgs::Marker>("/securityDistMarkersExt", 2);
  laser1_sub = nh->subscribe<sensor_msgs::LaserScan>("/scanMulti", 1, &SecurityMargin::laser1Callback, this);

  // Control flags
  status = 0;
  laserGot = false;
  cnt = 0;
  cnt2 = 0;

  red.a = 1;
  red.b = 0;
  red.g = 0;
  red.r = 1;

  green.a = 1;
  green.g = 1;
  green.b = 0;
  green.r = 0;

  blue.a = 1;
  blue.g = 0;
  blue.b = 1;
  blue.r = 0;

  markerIntFr.header.frame_id = base_link_frame;
  markerIntFr.header.stamp = ros::Time();
  markerIntFr.ns = "security_perimeter";
  markerIntFr.id = 1;
  markerIntFr.type = visualization_msgs::Marker::LINE_STRIP;
  markerIntFr.action = visualization_msgs::Marker::ADD;
  markerIntFr.lifetime = ros::Duration(2);
  markerIntFr.scale.x = 0.025;
  markerIntFr.color = green;

  markerExtFr = markerIntFr;

  markerExtFr.header.stamp = ros::Time();
  markerExtFr.id = 2;
}
// Mode 2: This is a ellipse centered at the robot frame, with the long axis along the x axis of the robot frame.
void SecurityMargin::buildElliptic()
{
  geometry_msgs::Point p;
  p.z = 0.3;

  markerIntFr.points.clear();
  markerExtFr.points.clear();

  for (auto i = 0; i < laserMsgLen; i++)
  {
    p.x = f * innerSecDistFront * cos(i / ((double)laserMsgLen) * 2 * M_PI);
    p.y = innerSecDistFront * sin(i / ((double)laserMsgLen) * 2 * M_PI);
    secArrayFr.push_back(sqrtf(p.x * p.x + p.y * p.y));

    if (((int)i % 50) == 0)
      markerIntFr.points.push_back(p);

    p.x *= extSecDistFront / innerSecDistFront;
    p.y *= extSecDistFront / innerSecDistFront;

    secArrayExtFr.push_back(sqrtf(p.x * p.x + p.y * p.y));
    if (((int)i % 50) == 0)
      markerExtFr.points.push_back(p);
  }
  // Last markers(identical to the first) I do this to close the ellipse marker
  p.x = f * innerSecDistFront;
  p.y = 0;
  markerIntFr.points.push_back(p);
  p.x *= extSecDistFront / innerSecDistFront;
  markerExtFr.points.push_back(p);
}
void SecurityMargin::publishRvizMarkers()
{
  switch (status)
  {
    case 0:
      markerIntFr.color = green;
      markerExtFr.color = green;
      // ROS_INFO(PRINTF_GREEN "0");
      break;
    case 1:
      markerIntFr.color = red;
      markerExtFr.color = red;
      // ROS_INFO(PRINTF_RED "1");
      break;
    case 2:
      markerIntFr.color = green;
      markerExtFr.color = red;
      // ROS_INFO(PRINTF_MAGENTA "2");
      break;
    case 3:
      markerIntFr.color = green;
      markerExtFr.color = red;
      // ROS_INFO(PRINTF_YELLOW "3");
      break;
    case 4:
      markerIntFr.color = blue;
      markerExtFr.color = blue;
      // ROS_INFO(PRINTF_BLUE"4");
      break;
    default:
      break;
  }

  marker_int_pub.publish(markerIntFr);
  marker_ext_pub.publish(markerExtFr);
}
// extPerimeter: boolean to select which ellipse to evaluate, 1 for ext and 0 for inner
// status int variable: values
// 0: All alright
// 1: algo dentro del margen interno
// 2: habia algo en el interno y ahora ha salido al externo
// 3: hay algo en el margen externo pero no ha entrado en el interno

//! Cuando algo entre la sucesion temporal de estados debe ser
//? 0->3->1->2->0
//! Si solo se tienen en cuenta los obstaculos que entran desde fuera tal vez se puedan evitar las reflexiones y
//! fantasmas de los laseres

bool SecurityMargin::checkObstacles()
{
  bool ret;
  cnt = cnt2 = 0;  // Todo lo relacionado con esto es para evitar fantasmas y reflexiones de los laseres
  /*
   * Si entra algo en el radio menor, esperar hasta que salga del mayor para volver a mover <- Esta es la idea principal
   *
   */
  for (auto i = 0; i < laserMsgLen; i++, cnt2++)
  {
    if (laserCPtr->ranges.at(i) < secArrayFr.at(i) && laserCPtr->ranges.at(i) > 0.2)  // laserCPtr->range_min)
    {
      // Algo hay dentro del margen interior
      if (cnt > 20)
      {  // forma basta de quitar fantasmas
        // Si llega a 10 significa que si debe haber algo de verda
        if ((status == 3 || status == 2))
          status = 1;

        ret = true;
        return ret;
      }
      else
      {
        cnt++;
      }
    }
    if (cnt2 > 300 && cnt < 20 && (status == 0 || status == 3))
      cnt = cnt2 = 0;
  }

  if (status == 1)
  {
    status = 2;
    ret = true;
    return ret;
  }

  cnt = cnt2 = 0;

  bool extObst = false;
  // 0: All alright
  // 1: algo dentro del margen interno
  // 2: habia algo en el interno y ahora ha salido al externo
  // 3: hay algo en el margen externo pero no ha entrado en el interno

  //! Cuando algo entre la sucesion temporal de estados debe ser
  //? 0->3->1->2->0
  //! Si solo se tienen en cuenta los obstaculos que entran desde fuera tal vez se puedan evitar las reflexiones y
  //! fantasmas de los laseres

  for (auto i = 0; i < laserMsgLen; i++, cnt2++)
  {  // Ahora si se ha llegao hasta aqui significa que no ha habido ningun obstaculo
    if (laserCPtr->ranges.at(i) < secArrayExtFr.at(i) && laserCPtr->ranges.at(i) > laserCPtr->range_min)
    {
      if (cnt > 20)
      {
        if (status == 2)
        {
          ret = true;
          return ret;
        }
        if (status == 0)
        {
          status = 3;
          ret = false;
          return ret;
        }
        extObst = true;
      }
      else
      {
        cnt++;
      }
    }
    if (cnt2 > 300 && cnt < 20)
      cnt = cnt2 = 0;
  }

  if (status == 3)
  {
    status = 0;
  }

  return false;
}
// This functions will use check obstacles function
// If something inside the dangerous area, returns false
// And it keeps returning false until the object has gone away from the security area(outside the outer ellipse)
bool SecurityMargin::canIMove()
{
  // refreshParams();
  bool ret = true;
  publishRvizMarkers();
  if (status == 4)  // Manual mode, ignoring security margin
  {
    return ret;
  }

  if (!laserGot)
  {
    ret = false;
    return ret;
  }

  if (checkObstacles())
  {
    ret = false;
  }

  if (stop_msg.data != !ret)
  {
    stop_msg.data = !ret;
    stop_pub.publish(stop_msg);
  }

  return ret;
}

}  // namespace Navigation
}  // namespace Upo