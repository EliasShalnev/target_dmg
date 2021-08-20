#include "target_damage/Model.h"

Model::Model(const ModelName& modelName)
    : m_modelName(modelName)
    , m_modelStates(m_nh, "/gazebo/model_states", 10)
{ }
