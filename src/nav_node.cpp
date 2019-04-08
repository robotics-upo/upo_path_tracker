
#include <ros/ros.h>
#include <math.h>
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>

#include <dynamic_reconfigure/server.h>
#include <arco_path_tracker/navConfig.h>

bool go_home;

void callback(arco_path_tracker::navConfig &config, uint32_t level);
void goHomeCb(const std_msgs::Bool &msg);
//Other stuff
tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv)
{
    string node_name = "nav_node";
    ros::init(argc, argv, node_name);
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

    ros::Rate loop_rate(40);

    //Dynamic reconfigure
    dynamic_reconfigure::Server<arco_path_tracker::navConfig> server;
  	dynamic_reconfigure::Server<arco_path_tracker::navConfig>::CallbackType f;

  	f = boost::bind(&callback, _1, _2);
  	server.setCallback(f);

    while (ros::ok())
    {
        ros::spinOnce();
        
        securityMargin.canIMove();

        despl.navigate(0);        

        loop_rate.sleep();
    }

    return 0;
}
void callback(arco_path_tracker::navConfig &config, uint32_t level) {
  /*ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.double_param, 
            config.size);*/
}
void goHomeCb(const std_msgs::Bool &msg)
{
    go_home = msg.data;
}