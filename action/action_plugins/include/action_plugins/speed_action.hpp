#ifndef INCLUDED_ACTION_PLUGINS_SPEED_ACTION_H
#define INCLUDED_ACTION_PLUGINS_SPEED_ACTION_H

#include <pluginlib/class_list_macros.hpp>

#include <scenario_actions/entity_action_base.hpp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_logger/logger.hpp>
#include <scenario_utility/scenario_utility.hpp>

namespace action_plugins
{

class SpeedAction // AbsoluteSpeedAction
  : public scenario_actions::EntityActionBase
{
  float value_;

public:
  SpeedAction();

  void configure(
    const YAML::Node&,
    const std::vector<std::string>,
    const std::shared_ptr<ScenarioAPI>&) override;

  auto run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    -> void override;
};

} // namespace action_plugins

#endif // INCLUDED_ACTION_PLUGINS_SPEED_ACTION_H

