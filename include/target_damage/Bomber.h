#pragma once

#include <geometry_msgs/Point.h>
#include <mavros_msgs/State.h>

#include "target_damage/SubMonitor.h"
#include "target_damage/Model.h"

class Observer;

class Bomber : public Model
{
public:
    using Ptr = std::shared_ptr<Bomber>;
    using ConstPtr = std::shared_ptr<Bomber const>;
    using BomberName = std::string;
    static constexpr char namePrefix[] = "bomber";
    static constexpr char ral_x6[] = "ral_x6";
    static constexpr char orlan[] = "orlan";

public:
    Bomber(const ModelName& modelName, const BomberName& bomberName, Observer& observer);
    Bomber(const Bomber&) = delete;
    Bomber& operator=(const Bomber&) = delete;
    ~Bomber() = default;

    bool isActive() const;

private:
    geometry_msgs::Point::ConstPtr evalHitPoint(const geometry_msgs::Point::ConstPtr& dropPoint);

    void dropPointCallback(const geometry_msgs::Point::ConstPtr& dropPoint);

private:
    const BomberName m_bomberName;
    
    Observer& m_observer;

    SubMonitor<mavros_msgs::State> m_uavState;
    ros::Subscriber m_dropPoint;
};

