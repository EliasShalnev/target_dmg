#include "target_damage/Target.h"


Target::Target(const ModelName& targetName)
    : Model(targetName)
    , m_damageSub(m_nh, "target_damage", 10)
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
    ROS_INFO_STREAM("Target \"" << m_modelName << "\"" << "parameters:" );
    m_damage.evalDamage(getCoordinates(), hitPoint);
}


Target::Damage::Damage(ModelName targetName) 
    : m_nh(targetName)
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


void Target::Damage::evalDamage(const geometry_msgs::Point::ConstPtr& targetCoord, 
                                const geometry_msgs::Point::ConstPtr& hitPoint) 
{
    const double B = 0.9; //поражающая способность
    const uint8_t R = 50; //радиус поражения
    const double k = 2; //коэффициент поражения - чем больше это коэф, тем меньше поражение
    constexpr double e = 2.71828; //экспонента

    const auto delta_x = hitPoint->x - targetCoord->x;
    const auto delta_y = hitPoint->y - targetCoord->y;

    const auto distanceToHit =  std::sqrt( std::pow(delta_x, 2) + std::pow(delta_y, 2) );

    const double temp = -(k* std::pow(distanceToHit, 2))/std::pow(R, 2);
    const auto damageDone = B*std::pow(e, temp);
    
    setDamage(m_currentDamage + damageDone );
    ROS_INFO_STREAM("- Damage: " << damageDone);
    ROS_INFO_STREAM("- Target point: x=" << targetCoord->x << " y=" << targetCoord->y);
}


void Target::Damage::publishDamage(const ros::TimerEvent& event) 
{
    target_dmg::TargetDamage targetDamage;
    targetDamage.damage = m_currentDamage;

    m_dmgPublisher.publish(targetDamage);
}