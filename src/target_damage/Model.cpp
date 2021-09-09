#include "target_damage/Model.h"

Model::Model(const ModelName& modelName)
    : m_modelName(modelName)
    , m_modelStates(m_nh, "/gazebo/model_states", 10)
{ }


bool Model::isActive() const
{
    for(auto model : m_modelStates.getMessage()->name)
    {
        if(model == m_modelName) { return true; }
    }
    return false;
}


geometry_msgs::Point::ConstPtr Model::getCoordinates() const
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


geometry_msgs::Vector3::ConstPtr Model::getMovementSpeed() const
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
