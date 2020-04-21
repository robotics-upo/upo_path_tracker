
#include <sfm_path_tracker/SFMController.hpp>

tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sfm_nav_node");
    ros::NodeHandle n;

    tf2_ros::TransformListener tfListener(tfBuffer);

    Upo::Navigation::SecurityMargin securityMargin;

    Upo::Navigation::SFMNav sfmcontroller(&n, &securityMargin, &tfBuffer);

    //SFM class subscribers
    ros::Subscriber path_sub = n.subscribe("/trajectory_tracker/local_input_trajectory", 1, &Upo::Navigation::SFMNav::trajectoryCb, &sfmcontroller);
    ros::Subscriber global_goal_sub = n.subscribe("/move_base_simple/goal", 1, &Upo::Navigation::SFMNav::globalGoalCb, &sfmcontroller);
    ros::Subscriber people = n.subscribe("/people", 1, &Upo::Navigation::SFMNav::trackedPersonCb, &sfmcontroller);
    ros::Subscriber imp = n.subscribe("/trajectory_tracker/impossible_to_find", 1, &Upo::Navigation::SFMNav::impossibleMoveCb, &sfmcontroller);
    ros::Subscriber local_goal_occ_sub = n.subscribe("/trajectory_tracker/local_goal_occupied", 1, &Upo::Navigation::SFMNav::occLocalGoalCb, &sfmcontroller);
 
    ros::Subscriber laser_sub = n.subscribe("/scanMulti", 1, &Upo::Navigation::SFMNav::laserReceived, &sfmcontroller);
 
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, &Upo::Navigation::SFMNav::odomReceived,&sfmcontroller);

    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, &Upo::Navigation::SFMNav::poseReceived,&sfmcontroller);

    ros::Rate loop_rate(40);

    while (ros::ok())
    {

        ros::spinOnce();
        sfmcontroller.navigate(loop_rate.cycleTime().toSec());
        loop_rate.sleep();
    }

    return 0;
}