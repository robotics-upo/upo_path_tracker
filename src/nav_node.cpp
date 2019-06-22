
#include <ros/ros.h>
#include <math.h>
#include <navigator/displacement.hpp>
#include <navigator/securityMargin.hpp>

#include <dynamic_reconfigure/server.h>
#include <arco_path_tracker/navConfig.h>


void callback(arco_path_tracker::navConfig &config, uint32_t level);

tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle n;

    tf2_ros::TransformListener tfListener(tfBuffer);

    SecurityMargin securityMargin(&n);

    Navigators::Displacement despl(&n, &securityMargin, &tfBuffer);
    //Displacement class subscribers
    ros::Subscriber path_sub = n.subscribe("/trajectory_tracker/local_input_trajectory", 1, &Navigators::Displacement::trajectoryCb, &despl);
    ros::Subscriber global_goal_sub = n.subscribe("/move_base_simple/goal", 1, &Navigators::Displacement::globalGoalCb, &despl);
    ros::Subscriber people = n.subscribe("/people", 1, &Navigators::Displacement::trackedPersonCb, &despl);
    ros::Subscriber imp = n.subscribe("/trajectory_tracker/impossible_to_find", 1, &Navigators::Displacement::impossibleMoveCb, &despl);
    ros::Subscriber local_goal_occ_sub = n.subscribe("/trajectory_tracker/local_goal_occupied", 1, &Navigators::Displacement::occLocalGoalCb, &despl);

    ros::Rate loop_rate(40);

    //Dynamic reconfigure
    dynamic_reconfigure::Server<arco_path_tracker::navConfig> server;
    dynamic_reconfigure::Server<arco_path_tracker::navConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    while (ros::ok())
    {

        ros::spinOnce();
        //securityMargin.canIMove();

        //despl.navigate();
        loop_rate.sleep();
    }

    return 0;
}
void callback(arco_path_tracker::navConfig &config, uint32_t level)
{
}
