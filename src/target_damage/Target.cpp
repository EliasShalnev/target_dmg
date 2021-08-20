#include "target_damage/Target.h"



Target::Target(const TargetName& targetName)
    : Model(targetName)
    , m_modelStates(m_nh, "/gazebo/model_states", 10)
    , m_damageSub(m_nh, "/sim_" + m_modelName + "/target_damage", 10)
    , m_damage(targetName)
{ }

bool Target::isActive() const
{
    auto damage = m_damageSub.getMessage()->damage;
    if(damage < 1) { return true; }
    return false;
}


void Target::evalDamage(const geometry_msgs::Point::ConstPtr& hitPoint) 
{
    const double B = 0.9; //поражающая способность
    const uint8_t R = 50; //радиус поражения
    const double k = 1; //TODO - понять что за коэффициент
    constexpr double e = 2.71828; //экспонента

    const auto delta_x = hitPoint->x - getCoordinates()->x;
    const auto delta_y = hitPoint->y - getCoordinates()->y;

    const auto distanceToHit =  std::sqrt( std::pow(delta_x, 2) + std::pow(delta_y, 2) );

    const double temp = -(k* std::pow(distanceToHit, 2))/std::pow(R, 2);
    const auto damageDone = B*std::pow(e, temp);
    
    m_damage.setDamage(m_damage.getDamage() + damageDone );
    ROS_INFO_STREAM(m_modelName << " was damaged: " << m_damage.getDamage() );
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


void Target::Damage::setDamage(const double& damage) 
{
    m_currentDamage = damage;

    target_dmg::TargetDamage targetDamage;
    targetDamage.damage = m_currentDamage;

    m_dmgPublisher.publish(targetDamage);

}


void Target::Damage::publishDamage(const ros::TimerEvent& event) 
{
    target_dmg::TargetDamage targetDamage;
    targetDamage.damage = m_currentDamage;

    m_dmgPublisher.publish(targetDamage);
}