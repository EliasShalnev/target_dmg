#pragma once 

#include <unordered_map>

#include <ros/subscriber.h>

#include <gazebo_msgs/ModelStates.h>

#include "target_damage/Model.h"


class Observer
{
public:
    Observer();
    ~Observer() = default;

    void evalDamage(const geometry_msgs::Point::ConstPtr& hitPoint);

private:
    /**
     * @brief checks if new target model is spawned and saves it in a targets store.
     * 
     * @param modelStates 
     */
    void checkRegisteredModels(const gazebo_msgs::ModelStates::ConstPtr& modelStates);

    void checkRegisteredNodes();

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_modelStatesSub;
    std::unordered_map<Model::ModelName, Model::Ptr> m_targets;
    std::unordered_map<Model::ModelName, Model::ConstPtr> m_bombers;
};
