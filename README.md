# target_dmg
This node compute damage estimation for each target. It finds all p3at models in /gazebo/model_states topic and interpret them as targets. The damage of each target is published in /sim_p3at$(number)/target_damage topic.

## target_dmg usage
The "drop_point" topic message of *bomber* UAV is a triger for targets damage estimation. So if you wants to bomb targets you supprose to publish drop point from some bomber.
