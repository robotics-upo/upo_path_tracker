#include <arco_path_tracker/ArcoPathTracker.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_node");
    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    Upo::Navigation::ArcoPathTracker despl(&tfBuffer);
    //Displacement class subscribers
    ros::Subscriber path_sub = n.subscribe("/local_planner_node/local_path", 1, &Upo::Navigation::ArcoPathTracker::trajectoryCb, &despl);

    ros::Rate loop_rate(20);

    //Dynamic reconfigure
    dynamic_reconfigure::Server<upo_path_tracker::ArcoPathTrackerConfig> server;
    dynamic_reconfigure::Server<upo_path_tracker::ArcoPathTrackerConfig>::CallbackType f;

    f = boost::bind(&Upo::Navigation::ArcoPathTracker::dynReconfCb, &despl, _1, _2);
    server.setCallback(f);

    while (ros::ok())
    {
        ros::spinOnce();

        despl.navigate();
        loop_rate.sleep();
    }

    return 0;
}