

#include "mpc_pkg/SFC_generate_cls.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    // ROS
    ros::init(argc, argv, "SFC_generate_node");
    ROS_INFO("\033[1;33m SFC generate node initialized! \033[0m");
    SFCGenerateNode sfc_generate_node;

    ros::Rate rate(10);

    // while(ros::ok())
    // {
    //     rate.sleep();
    // }
    ros::spin();

    return 0;
}