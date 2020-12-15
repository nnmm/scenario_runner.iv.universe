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

#include "entity_plugins/ego_entity.hpp"

namespace entity_plugins
{

EgoEntity::EgoEntity()
: scenario_entities::EntityBase{"Ego"}
{}

bool EgoEntity::configure(
  const YAML::Node & entity,
  const std::shared_ptr<ScenarioAPI> & api)
try
{
  if (!EntityBase::configure(entity, api)) {
    return false;
  }

  urdf_ = read_optional<std::string>(entity, "Urdf");

  initial_frame_id_ =
    read_optional<std::string>(
    entity, "InitialFrameId", "ego-initial-pose");

  if (not api_->setEgoCarName(name_)) {
    SCENARIO_ERROR_THROW(CATEGORY(), "Failed to set ego-car-name.");
  }
  return true;
} catch (...) {
  SCENARIO_ERROR_RETHROW(
    CATEGORY(),
    "Failed to configure entity named '" << name_ << "' of type " << type_ << ".");
}

bool EgoEntity::init()
try
{
  if (const auto speed_node {init_entity_["InitialStates"]["Speed"]}) {
    const auto speed {speed_node.as<float>()};
    if (not api_->setMaxSpeed(speed)) {
      SCENARIO_ERROR_THROW(CATEGORY(), "Failed to set max-speed.");
    }
  }

  if (const auto initial_speed {init_entity_["InitialStates"]["InitialSpeed"]}) {
    if (not api_->sendStartVelocity(initial_speed.as<float>())) {
      SCENARIO_ERROR_THROW(CATEGORY(), "Failed to send start-velicity.");
    }
  } else {
    if (not api_->sendStartVelocity(0)) {
      SCENARIO_ERROR_THROW(CATEGORY(), "Failed to send start-velicity.");
    }
  }

  call_with_essential(
    init_entity_, "InitialStates", [&](const auto & node) mutable
    {
      const auto pose {read_essential<geometry_msgs::msg::Pose>(node, "Pose")};

      if (not api_->sendStartPoint(
        pose, true, read_optional<std::string>(
          node, "Shift",
          "Center")))
      {
        SCENARIO_ERROR_THROW(CATEGORY(), "Failed to send start-point.");
      }

      if (not api_->setFrameId(initial_frame_id_, pose)) {
        SCENARIO_ERROR_THROW(CATEGORY(), "Failed to set frame-id.");
      }
    });

  call_with_optional(
    init_entity_, "Actions", [&](const auto & node) mutable
    {
      initial_action_manager_ptr_ =
      std::make_shared<scenario_actions::ActionManager>(
        node, std::vector<std::string> {name_}, api_);

      initial_action_manager_ptr_->run(nullptr);
    });

  return true;
} catch (...) {
  SCENARIO_ERROR_RETHROW(
    CATEGORY(),
    "Failed to initialize entity named '" << name_ << "' of type " << type_ << ".");
}

} // namespace entity_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(entity_plugins::EgoEntity, scenario_entities::EntityBase)
