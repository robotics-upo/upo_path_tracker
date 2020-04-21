
#include <ros/ros.h>
#include <math.h>
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>
void callback(upo_path_tracker::navConfig &config, uint32_t level);

tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle n;

    tf2_ros::TransformListener tfListener(tfBuffer);

    Navigators::Displacement despl(&tfBuffer);
    //Displacement class subscribers
    ros::Subscriber path_sub = n.subscribe("/local_planner_node/local_path", 1, &Navigators::Displacement::trajectoryCb, &despl);

    ros::Rate loop_rate(20);

    //Dynamic reconfigure
    dynamic_reconfigure::Server<upo_path_tracker::navConfig> server;
    dynamic_reconfigure::Server<upo_path_tracker::navConfig>::CallbackType f;

    f = boost::bind(&Navigators::Displacement::dynReconfCb, &despl, _1, _2);
    server.setCallback(f);

    while (ros::ok())
    {
        ros::spinOnce();

        despl.navigate();
        loop_rate.sleep();
    }

    return 0;
}