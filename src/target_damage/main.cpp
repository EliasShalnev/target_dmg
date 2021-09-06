#include <ros/init.h>

#include "target_damage/Observer.h"

const std::string nodeName = "target_dmg";

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    Observer observer;

    ros::spin();
    return 0;
}