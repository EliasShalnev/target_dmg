#pragma once

#include <string>
#include <memory>

#include <gazebo_msgs/ModelStates.h>
#include "target_dmg/TargetDamage.h"

#include "target_damage/Model.h"
// #include "target_damage/SubMonitor.h"

class Target : public Model
{
public:
    using Ptr = std::shared_ptr<Target>;
    using ConstPtr = std::shared_ptr<Target const>;
    using TargetName = std::string;
    static constexpr char namePrefix[] = "p3at";

public:
    Target(const TargetName& targetName);
    Target(const Target&) = delete;
    Target& operator=(const Target&) = delete;
    ~Target() = default;

    /**
     * @brief Target assume to be active while model with "m_tagetName" is published
     *        in "/gazebo/model_states" topic
     * 
     * @return true if active
     * @return false if isn't active 
     */
    bool isActive() const override;

    geometry_msgs::Point::ConstPtr getCoordinates() const override;

    geometry_msgs::Vector3::ConstPtr getMovementSpeed() const override;

private:
    SubMonitor<target_dmg::TargetDamage> m_currentDamage;
    class Damage
    {
    public:
        Damage(TargetName targetName);
        ~Damage() = default;

    private:
        void publishDamage(const ros::TimerEvent& event);

    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_dmgPublisher;
        ros::Timer m_publisherTimer;
        float m_currentDamage = 0.0;
    } m_damage;
};
