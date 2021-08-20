#include "target_damage/ModelsObserver.h"

#include "target_damage/Target.h"
#include "target_damage/Bomber.h"


ModelsObserver::ModelsObserver()
    : m_modelStatesSub( m_nh.subscribe<gazebo_msgs::ModelStates>
                        ("/gazebo/model_states", 10, &ModelsObserver::checkRegisteredModels, this) )
{ }


void ModelsObserver::checkRegisteredModels(const gazebo_msgs::ModelStates::ConstPtr& modelStates) 
{
    for(auto model : modelStates->name)
    {
        if( !checkModel(model, Target::namePrefix, m_targets) ) { continue; }
        checkModel(model, Bomber::namePrefix, m_bombers);
    }
}


bool ModelsObserver::checkModel(const Model::ModelName& model,
                                const std::string& modelPrefix,
                                std::unordered_map<Model::ModelName, Model::ConstPtr>& modelStore) const
{
    if( model.find(modelPrefix) == std::string::npos ) { return false; } //не наш клиент (исчи дальше)
    if( modelStore.find(model) != modelStore.end() ) { return true; } //наш клиент, но мы его уже мониторим

    if(modelPrefix == Target::namePrefix)
    {
        ROS_INFO_STREAM("New target \"" << model << "\" was founded.");
        modelStore.emplace( model, new Target(model) );
    }
    if(modelPrefix == Bomber::namePrefix)
    {
        ROS_INFO_STREAM("New bomber \"" << model << "\" was founded.");
        modelStore.emplace( model, new Bomber(model) );

    }

    //таймер для удаления модели из хранилища, если она не активна
    ros::Timer* clearTimer = new ros::Timer();
    *clearTimer = m_nh.createTimer(ros::Duration(1), 
                                   [model, &modelStore, clearTimer](const ros::TimerEvent& event)
    {
        auto modelIt = modelStore.find(model);
        
        if( !modelIt->second->isActive() )
        {
            clearTimer->stop();
            delete clearTimer;
            modelStore.erase(modelIt);
            ROS_INFO_STREAM("Model \"" << model << "\" was removed.");
        }
    });
    clearTimer->start();

    return true;
}
