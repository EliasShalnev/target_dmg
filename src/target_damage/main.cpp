#include <ros/init.h>

#include "target_damage/ModelsObserver.h"

const std::string nodeName = "target_dmg";

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);

    ModelsObserver targetObserver;

    ros::spin();
    return 0;
}