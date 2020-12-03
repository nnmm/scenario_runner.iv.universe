#ifndef INCLUDED_ACTION_PLUGINS_FAULT_INJECTION_ACTION_H
#define INCLUDED_ACTION_PLUGINS_FAULT_INJECTION_ACTION_H

#include <scenario_actions/entity_action_base.hpppp>
#include <scenario_intersection/intersection_manager.hpp>
#include <scenario_logger/logger.hpp>
#include <scenario_utility/scenario_utility.hpp>

namespace action_plugins
{

class FaultInjectionAction
  : public scenario_actions::EntityActionBase
{
  std::string target_node_;

public:
  FaultInjectionAction()
    : EntityActionBase {"FaultInjection"}
  {}

  void configure(
    const YAML::Node&,
    const std::vector<std::string>,
    const std::shared_ptr<ScenarioAPI>&) override;

  void run(const std::shared_ptr<scenario_intersection::IntersectionManager>&) override;
};

} // namespace action_plugins

#endif // INCLUDED_ACTION_PLUGINS_FAULT_INJECTION_ACTION_H
