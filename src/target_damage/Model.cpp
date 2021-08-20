#include "target_damage/Model.h"

Model::Model(const ModelName& modelName)
    : m_nh(modelName)
    , m_modelName(modelName)
{ }
