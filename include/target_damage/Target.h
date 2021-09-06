#pragma once

#include <string>
#include <memory>

#include "target_dmg/TargetDamage.h"

#include "target_damage/Model.h"

class Target : public Model
{
public:
    using Ptr = std::shared_ptr<Target>;
    using ConstPtr = std::shared_ptr<Target const>;
    static constexpr char namePrefix[] = "p3at";

public:
    Target(const ModelName& targetName);
    Target(const Target&) = delete;
    Target& operator=(const Target&) = delete;
    ~Target() = default;

    /**
     * @brief Target assume to be active while target damage is lesser that 0
     *        
     * 
     * @return true if active
     * @return false if isn't active 
     */
    bool isActive() const override;

    void evalDamage(const geometry_msgs::Point::ConstPtr& hitPoint);

private:
    SubMonitor<target_dmg::TargetDamage> m_damageSub;

    class Damage
    {
    public:
        Damage(ModelName targetName);
        ~Damage() = default;

        void evalDamage(const geometry_msgs::Point::ConstPtr &targetCoord, 
                        const geometry_msgs::Point::ConstPtr& hitPoint);

    private:
        void setDamage(const double& damage);
        
        void publishDamage(const ros::TimerEvent& event);

    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_dmgPublisher;
        ros::Timer m_publisherTimer;
        double m_currentDamage = 0.0;
    } m_damage;
};
