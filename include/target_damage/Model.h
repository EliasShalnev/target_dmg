#pragma once

#include <gazebo_msgs/ModelStates.h>

#include "target_damage/SubMonitor.h"

class Model
{
public:
    using Ptr = std::shared_ptr<Model>;
    using ConstPtr = std::shared_ptr<Model const>;
    using ModelName = std::string;

public:
    Model(const ModelName& modelName);
    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
    virtual ~Model() = default;

    /**
     * @brief Model assume to be active while model with "m_modelName" is published
     *        in "/gazebo/model_states" topic
     * 
     * @return true is active
     * @return false isn't active 
     */
    virtual bool isActive() const;

    geometry_msgs::Point::ConstPtr getCoordinates() const;

    geometry_msgs::Vector3::ConstPtr getMovementSpeed() const;

protected:
    const ModelName m_modelName;
    ros::NodeHandle m_nh;
    SubMonitor<gazebo_msgs::ModelStates> m_modelStates;
};