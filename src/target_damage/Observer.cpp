#include "target_damage/Observer.h"

#include <ros/master.h>

#include "target_damage/Target.h"
#include "target_damage/Bomber.h"

#include "target_damage/NSParser.h"


Observer::Observer()
    : m_modelStatesSub( m_nh.subscribe<gazebo_msgs::ModelStates>
                        ("/gazebo/model_states", 10, &Observer::checkRegisteredModels, this) )
{ }


void Observer::evalDamage(const geometry_msgs::Point::ConstPtr& hitPoint) 
{
    for(auto [_, targetModel] : m_targets)
    {
        auto target = dynamic_cast<Target*>( targetModel.get() );
        target->evalDamage(hitPoint);
    }
}


void Observer::checkRegisteredModels(const gazebo_msgs::ModelStates::ConstPtr& modelStates) 
{
    for(auto model : modelStates->name)
    {
        if( model.find(Target::namePrefix) == std::string::npos ) { continue; } //не наш клиент (исчи дальше)
        if( m_targets.find(model) != m_targets.end() ) { continue; } //наш клиент, но мы его уже мониторим

        ROS_INFO_STREAM("New target \"" << model << "\" was founded.");
        m_targets.emplace( model, new Target(model) );

        //таймер для удаления модели из хранилища, если она не активна
        // ros::Timer* clearTimer = new ros::Timer();
        // *clearTimer = m_nh.createTimer(ros::Duration(1), 
        //                                [this, model, clearTimer](const ros::TimerEvent& event)
        // {
        //     auto modelIt = m_targets.find(model);
            
        //     if( !modelIt->second->isActive() )
        //     {
        //         clearTimer->stop();
        //         delete clearTimer;
        //         m_targets.erase(modelIt);
        //         ROS_INFO_STREAM("Target \"" << model << "\" was removed.");
        //     }
        // });
        // clearTimer->start();
    }

    checkRegisteredNodes();
}


void Observer::checkRegisteredNodes() 
{
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);

    for(auto node : nodes)
    {
        NSParser nsParser(node);
        if(nsParser.size() != 2) { continue; }

        if( *(nsParser.end()-1) == std::string("mavros") ) { continue; }
        
        if(nsParser.begin()->find(Bomber::namePrefix) == std::string::npos) { continue; }
        Bomber::BomberName bomberName = nsParser.begin()->erase(0, 1);
        if( m_bombers.find(bomberName) != m_bombers.end() ) { continue; }
        
        ROS_INFO_STREAM("New bomber \"" << bomberName<< "\" was founded.");
        m_bombers.emplace( bomberName, new Bomber( bomberName, *this ) );

        //таймер для удаления модели из хранилища, если она не активна
        // ros::Timer* clearTimer = new ros::Timer();
        // *clearTimer = m_nh.createTimer(ros::Duration(1), 
        //                                [this, bomberName, clearTimer](const ros::TimerEvent& event)
        // {
        //     auto modelIt = m_bombers.find(bomberName);
            
        //     if( !modelIt->second->isActive() )
        //     {
        //         clearTimer->stop();
        //         delete clearTimer;
        //         m_bombers.erase(modelIt);
        //         ROS_INFO_STREAM("Bomber \"" << bomberName << "\" was removed.");
        //     }
        // });
        // clearTimer->start();
    }
}
