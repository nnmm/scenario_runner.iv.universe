// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDED_ACTION_PLUGINS_ENABLE_ACTION_H
#define INCLUDED_ACTION_PLUGINS_ENABLE_ACTION_H

#include "scenario_actions/entity_action_base.hpppp"
#include "scenario_intersection/intersection_manager.hpp"
#include "scenario_logger/logger.hpp"
#include "scenario_utility/scenario_utility.hpp"

namespace action_plugins
{

class EnableAction
  : public scenario_actions::EntityActionBase
{
  std::string name_;

  boost::optional<bool> automatic_emergency_braking_;

public:
  EnableAction();

  void configure(
    const YAML::Node&,
    const std::vector<std::string>,
    const std::shared_ptr<ScenarioAPI>&) override;

  void run(
    const std::shared_ptr<scenario_intersection::IntersectionManager>&)
    override;
};

} // namespace action_plugins

#endif // INCLUDED_ACTION_PLUGINS_ENABLE_ACTION_H
