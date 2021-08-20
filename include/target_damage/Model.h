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

    virtual bool isActive() const = 0;

    // virtual geometry_msgs::Point::ConstPtr getCoordinates() const = 0;

    // virtual geometry_msgs::Vector3::ConstPtr getMovementSpeed() const = 0;

protected:
    ros::NodeHandle m_nh;
    const ModelName m_modelName;
};

