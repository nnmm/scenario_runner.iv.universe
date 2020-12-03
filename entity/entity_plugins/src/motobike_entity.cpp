#include <entity_plugins/motorbike_entity.hpp>

namespace entity_plugins
{

MotorBikeEntity::MotorBikeEntity()
  : scenario_entities::EntityBase {"MotorBike"}
{}

} // namespace entity_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(entity_plugins::MotorBikeEntity, scenario_entities::EntityBase)

