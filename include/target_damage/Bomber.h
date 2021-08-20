#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/State.h>

#include "target_damage/Model.h"
#include "target_damage/Observer.h"

class Bomber : public Model
{
public:
    using Ptr = std::shared_ptr<Bomber>;
    using ConstPtr = std::shared_ptr<Bomber const>;
    using BomberName = std::string;
    static constexpr char namePrefix[] = "bomber";

public:
    Bomber(const BomberName& bomberName, Observer& observer);
    Bomber(const Bomber&) = delete;
    Bomber& operator=(const Bomber&) = delete;
    ~Bomber() = default;

    bool isActive() const override;

private:
    geometry_msgs::Point::ConstPtr evalHitPoint(const geometry_msgs::Point::ConstPtr& dropPoint);

    void dropPointCallback(const geometry_msgs::Point::ConstPtr& dropPoint);

private:
    Observer& m_observer;
    SubMonitor<geometry_msgs::PoseStamped> m_cooridnates;
    SubMonitor<geometry_msgs::TwistStamped> m_movementSpeed;
    SubMonitor<mavros_msgs::State> m_uavState;
    ros::Subscriber m_dropPoint;
};

