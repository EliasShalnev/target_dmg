#include "target_damage/Observer.h"

#include <ros/master.h>

#include "target_damage/Target.h"
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
    checkRegisteredTargets(modelStates);

    checkRegisteredBombers(modelStates);
}


void Observer::checkRegisteredTargets(const gazebo_msgs::ModelStates::ConstPtr& modelStates) 
{
    for(auto model : modelStates->name)
    {
        if( model.find(Target::namePrefix) == std::string::npos ) { continue; } //не наш клиент (исчи дальше)
        if( m_targets.find(model) != m_targets.end() ) { continue; } //наш клиент, но мы его уже мониторим

        ROS_INFO_STREAM("New target \"" << model << "\" was found.");
        m_targets.emplace( model, new Target(model) );

        //TODO - раскомментировать, когда будет корректное удаление моделей из Gazebo
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
}


void Observer::checkRegisteredBombers(const gazebo_msgs::ModelStates::ConstPtr& modelStates) 
{
    auto foundBombers = checkRegisteredNodes();

    for(auto bomber : foundBombers)
    {
        auto id = bomber.substr( strlen(Bomber::namePrefix) );
        for(auto model : modelStates->name)
        {
            std::string modelId;
            if( model.find(Bomber::ral_x6) != std::string::npos )
            {
                modelId = model.substr( strlen(Bomber::ral_x6) );
            }
            else if( model.find(Bomber::orlan) != std::string::npos )
            {
                modelId = model.substr( strlen(Bomber::orlan) );
            }
            else { continue; }
            if(modelId == id) 
            { 
                ROS_INFO_STREAM("New bomber \"" << bomber<< "\" was found with"
                                " \"" << model << "\" model.");

                m_bombers.emplace( bomber, new Bomber(model, bomber, *this) );
            }
        }
    }
}


std::vector<Bomber::BomberName> Observer::checkRegisteredNodes() 
{
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);

    std::vector<Bomber::BomberName> foundBombers;

    for(auto node : nodes)
    {
        NSParser nsParser(node);
        if(nsParser.size() != 2) { continue; }

        if( *(nsParser.end()-1) == std::string("mavros") ) { continue; }
        
        if(nsParser.begin()->find(Bomber::namePrefix) == std::string::npos) { continue; }
        Bomber::BomberName bomberName = nsParser.begin()->erase(0, 1);
        if( m_bombers.find(bomberName) != m_bombers.end() ) { continue; }
        
        foundBombers.emplace_back(bomberName);

        //TODO - раскомментировать, когда будет корректное удаление моделей из Gazebo
        // таймер для удаления модели из хранилища, если она не активна
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

    return foundBombers;
}
