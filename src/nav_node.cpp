
#include <ros/ros.h>
#include <math.h>
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>

#include <dynamic_reconfigure/server.h>
#include <arco_path_tracker/navConfig.h>

//Debug libs
#include <ctime>
#include <std_msgs/Int32.h>
#include <sys/timeb.h>

//#define Debug

//Homing under construction
bool go_home;

void callback(arco_path_tracker::navConfig &config, uint32_t level);
void goHomeCb(const std_msgs::Bool &msg);

tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle n;

    tf2_ros::TransformListener tfListener(tfBuffer);
    
    SecurityMargin securityMargin(&n);
    Navigators::Displacement despl(&n, &securityMargin, &tfBuffer);

    ros::Subscriber laser_sub = n.subscribe("/scanFront", 1, &SecurityMargin::laser1Callback, &securityMargin);
    ros::Subscriber laser_sub2 = n.subscribe("/scanBackFiltered", 1, &SecurityMargin::laser2Callback, &securityMargin);
    ros::Subscriber path_sub = n.subscribe("/trajectory_tracker/local_input_trajectory", 1, &Navigators::Displacement::trajectoryCb, &despl);
    ros::Subscriber global_goal_sub = n.subscribe("/move_base_simple/goal", 1, &Navigators::Displacement::globalGoalCb, &despl);
    //Homing under construction
    ros::Subscriber goHome = n.subscribe("/trajectory_tracker/go_home", 1, goHomeCb);
    ros::Subscriber people = n.subscribe("/people", 1, &Navigators::Displacement::trackedPersonCb, &despl);
    ros::Subscriber imp = n.subscribe("/trajectory_tracker/impossible_to_find",1, &Navigators::Displacement::impossibleMoveCb, &despl);

#ifdef DEBUG
    ros::Publisher plan_time = n.advertise<std_msgs::Int32>("/nav_loop_times", 1000);
    float seconds, milliseconds;
    std_msgs::Int32 msg;
    struct timeb startT, finishT;
#endif
    ros::Rate loop_rate(40);

    //Dynamic reconfigure
    dynamic_reconfigure::Server<arco_path_tracker::navConfig> server;
  	dynamic_reconfigure::Server<arco_path_tracker::navConfig>::CallbackType f;

  	f = boost::bind(&callback, _1, _2);
  	server.setCallback(f);

    while (ros::ok())
    {
    
#ifdef DEBUG
      ftime(&startT);
#endif
      ros::spinOnce();
    
      securityMargin.canIMove();

      despl.navigate();      

#ifdef DEBUG
      ftime(&finishT);
      seconds = finishT.time - startT.time - 1;
      milliseconds = (1000 - startT.millitm) + finishT.millitm;
      msg.data = (milliseconds + seconds * 1000);
      plan_time.publish(msg);
#endif
      loop_rate.sleep();
    }

    return 0;
}
void callback(arco_path_tracker::navConfig &config, uint32_t level) {

}
void goHomeCb(const std_msgs::Bool &msg)
{
    go_home = msg.data;
}