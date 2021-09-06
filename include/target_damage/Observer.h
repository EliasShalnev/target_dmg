#pragma once 

#include <unordered_map>

#include <ros/subscriber.h>

#include <gazebo_msgs/ModelStates.h>

#include "target_damage/Model.h"
#include "target_damage/Bomber.h"


class Observer
{
public:
    Observer();
    Observer(const Observer&) = delete;
    Observer& operator=(const Observer&) = delete;
    ~Observer() = default;

    void evalDamage(const geometry_msgs::Point::ConstPtr& hitPoint);

private:
    /**
     * @brief /gazebo/model_states callback
     * 
     * @param modelStates 
     */
    void checkRegisteredModels(const gazebo_msgs::ModelStates::ConstPtr& modelStates);

    /**
     * @brief checks if new target model is spawned and saves it in a targets store.
     * 
     * @param modelStates 
     */
    void checkRegisteredTargets(const gazebo_msgs::ModelStates::ConstPtr& modelStates);

    /**
     * @brief find new bomber model is spawned and saves it in a targets store.
     * 
     * @param modelStates 
     */
    void checkRegisteredBombers(const gazebo_msgs::ModelStates::ConstPtr& modelStates);
    
    /**
     * @brief checks if new bomber node is spawned and saves it in a bobmbers store.
     * 
     */
    std::vector<Bomber::BomberName> checkRegisteredNodes();

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_modelStatesSub;
    std::unordered_map<Model::ModelName, Model::Ptr> m_targets;
    std::unordered_map<Bomber::BomberName, Bomber::ConstPtr> m_bombers;
};
