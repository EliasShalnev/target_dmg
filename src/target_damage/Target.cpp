#include "target_damage/Target.h"



Target::Target(const TargetName& targetName)
    : Model(targetName)
    , m_currentDamage(m_nh, "/sim_" + m_modelName + "/target_damage", 10)
    , m_damage(targetName)
{ }

bool Target::isActive() const
{
    auto damage = m_currentDamage.getMessage()->damage;
    if(damage < 1) { return true; }
    return false;
}


geometry_msgs::Point::ConstPtr Target::getCoordinates() const 
{
    auto models = m_modelStates.getMessage()->name;
    for(unsigned int i=0; i<models.size(); ++i)
    {
        if(models[i] == m_modelName)
        {   
            auto pose = new geometry_msgs::Point(m_modelStates.getMessage()->pose[i].position);
            return geometry_msgs::Point::ConstPtr(pose);
        }
    }
    return nullptr;
}


geometry_msgs::Vector3::ConstPtr Target::getMovementSpeed() const
{
    auto models = m_modelStates.getMessage()->name;
    for(unsigned int i=0; i<models.size(); ++i)
    {
        if(models[i] == m_modelName)
        {   
            auto ms = new geometry_msgs::Vector3(m_modelStates.getMessage()->twist[i].linear);
            return geometry_msgs::Vector3::ConstPtr(ms);
        }
    }
    return nullptr;
}


Target::Damage::Damage(TargetName targetName) 
    : m_nh("/sim_" + targetName)
    , m_dmgPublisher( m_nh.advertise<target_dmg::TargetDamage>("target_damage", 10) )
    , m_publisherTimer( m_nh.createTimer( ros::Duration(0.5), &Target::Damage::publishDamage, this) )
{ }


void Target::Damage::publishDamage(const ros::TimerEvent& event) 
{
    target_dmg::TargetDamage targetDamage;
    targetDamage.damage = m_currentDamage;

    m_dmgPublisher.publish(targetDamage);
}