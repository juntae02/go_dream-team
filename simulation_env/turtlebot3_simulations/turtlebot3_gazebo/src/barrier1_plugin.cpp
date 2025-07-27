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

// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0

// #include "turtlebot3_gazebo/barrier1_plugin.hpp"

// #include <iostream>

// namespace gazebo
// {
// Barrier1Plugin::Barrier1Plugin()
// : move_cycle(0.5),  // [설정] 이동 주기 (사용 안 함: 참고용)
//   going_to_pose2(true)  // [추가] 처음에는 pose2로 이동 시작
// {
// }

// void Barrier1Plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
// {
//   this->model = _model;
//   this->world = _model->GetWorld();

//   std::cout << "[Barrier1Plugin] Loaded for model: " << this->model->GetName() << std::endl;

//   // [추가] 초기 pose 설정
//   this->pose_1 = ignition::math::Pose3d(
//     ignition::math::Vector3d(0.010181, 0.760888, 0.10),
//     ignition::math::Quaterniond(0.0, 0.0, 1.58756));
//   this->pose_2 = ignition::math::Pose3d(
//     ignition::math::Vector3d(-0.640166, 0.749985, 0.10),
//     ignition::math::Quaterniond(-0.000001, 0.000004, 1.587558));

//   this->last_time = this->world->SimTime();

//   // [중요] 중력 영향 제거
//   this->model->SetGravityMode(false);

//   // [중요] 월드 업데이트 시 실행될 함수 연결
//   this->update_connection = event::Events::ConnectWorldUpdateBegin(
//     std::bind(&Barrier1Plugin::OnUpdate, this));
// }

// void Barrier1Plugin::OnUpdate()
// {
//   common::Time current_time = this->world->SimTime();
//   // double dt = (current_time - this->last_time).Double();

//   // [설정] 이동 속도 (너무 크면 순간이동처럼 보임)
//   double speed = 0.05;  // m/s

//   // 현재 위치, 목표 위치
//   ignition::math::Vector3d current_pos = this->model->WorldPose().Pos();
//   ignition::math::Vector3d target_pos = going_to_pose2 ? pose_2.Pos() : pose_1.Pos();
//   ignition::math::Vector3d move_dir = target_pos - current_pos;
  
//   move_dir.Z() = 0.0;

//   // 목표 위치에 거의 도착하면 방향 반전
//   if (move_dir.Length() < 0.01) {
//     going_to_pose2 = !going_to_pose2;
//     move_dir = ignition::math::Vector3d(0, 0, 0);
//   } else {
//     move_dir.Normalize();
//   }

//   // [핵심] 링크를 가져와 속도 적용 → 슬리퍼 끄는 느낌 구현
//   auto link = this->model->GetLink("link");  // 실제 sdf에 정의된 링크 이름
//   if (link) {
//     link->SetLinearVel(move_dir * speed);
//   }

//   this->last_time = current_time;
// }
// }  // namespace gazebo



#include "turtlebot3_gazebo/barrier1_plugin.hpp"

#include <iostream>

namespace gazebo
{
Barrier1Plugin::Barrier1Plugin()
: move_cycle(0.5),
  going_to_pose2(true),
  is_waiting(false),                        // [추가]
  wait_duration(1.0)                        // [추가] 멈춤 시간 2초
{
}

void Barrier1Plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();

  std::cout << "[Barrier1Plugin] Loaded for model: " << this->model->GetName() << std::endl;

  this->pose_1 = ignition::math::Pose3d(
    ignition::math::Vector3d(0.010181, 0.760888, 0.10),
    ignition::math::Quaterniond(0.0, 0.0, 1.58756));
  this->pose_2 = ignition::math::Pose3d(
    ignition::math::Vector3d(-1.040166, 0.749985, 0.10),
    ignition::math::Quaterniond(-0.000001, 0.000004, 1.587558));

  this->last_time = this->world->SimTime();
  this->wait_start_time = this->last_time;  // [추가]

  this->model->SetGravityMode(false);

  this->update_connection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&Barrier1Plugin::OnUpdate, this));
}

void Barrier1Plugin::OnUpdate()
{
  common::Time current_time = this->world->SimTime();

  auto link = this->model->GetLink("link");
  if (!link) return;

  ignition::math::Vector3d current_pos = this->model->WorldPose().Pos();
  ignition::math::Vector3d target_pos = going_to_pose2 ? pose_2.Pos() : pose_1.Pos();
  ignition::math::Vector3d move_dir = target_pos - current_pos;
  move_dir.Z() = 0.0;

  double speed = 0.07;  // m/s

  if (is_waiting) {
    // [추가] 대기 중일 경우 일정 시간 기다림
    if ((current_time - wait_start_time).Double() >= wait_duration) {
      is_waiting = false;
    } else {
      link->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
      return;
    }
  }

  if (move_dir.Length() < 0.01) {
    // [수정] 도착 시 대기 시작
    going_to_pose2 = !going_to_pose2;
    is_waiting = true;
    wait_start_time = current_time;
    link->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
    return;
  }

  move_dir.Normalize();
  link->SetLinearVel(move_dir * speed);

  this->last_time = current_time;
}
}  // namespace gazebo
