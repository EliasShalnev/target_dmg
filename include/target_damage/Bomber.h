#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "target_damage/Model.h"

class Bomber : public Model
{
public:
    using Ptr = std::shared_ptr<Bomber>;
    using ConstPtr = std::shared_ptr<Bomber const>;
    using BomberName = std::string;
    static constexpr char namePrefix[] = "bomber";

public:
    Bomber(const BomberName& bomberName);
    Bomber(const Bomber&) = delete;
    Bomber& operator=(const Bomber&) = delete;
    ~Bomber() = default;

    bool isActive() const override;

    void getDropPoint();

    geometry_msgs::Point::ConstPtr getCoordinates() const override;

    geometry_msgs::Vector3::ConstPtr getMovementSpeed() const override;

private:
    SubMonitor<geometry_msgs::PoseStamped> m_cooridnates;
    SubMonitor<geometry_msgs::TwistStamped> m_movementSpeed;
};

