// Copyright 2025 ROBOTIS CO., LTD.
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
//
// Author: Hyungyu Kim

#ifndef TURTLEBOT3_GAZEBO__BARRIER1_PLUGIN_HPP_
#define TURTLEBOT3_GAZEBO__BARRIER1_PLUGIN_HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class Barrier1Plugin : public ModelPlugin
{
public:
  Barrier1Plugin();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate();

private:
  double move_cycle;
  bool going_to_pose2;

  common::Time last_time;
  event::ConnectionPtr update_connection;
  ignition::math::Pose3d pose_1;
  ignition::math::Pose3d pose_2;
  ignition::math::Vector3d direction; 
  physics::ModelPtr model;
  physics::WorldPtr world;

  bool is_waiting;
  common::Time wait_start_time;
  double wait_duration;

};
GZ_REGISTER_MODEL_PLUGIN(Barrier1Plugin);
}  // namespace gazebo
#endif  // TURTLEBOT3_GAZEBO__TRAFFIC_BAR_PLUGIN_HPP_
