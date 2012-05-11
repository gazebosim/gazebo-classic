/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef __GAZEBO_GRIPPER_PLUGIN_HH__
#define __GAZEBO_GRIPPER_PLUGIN_HH__

#include <map>
#include <vector>
#include <string>

#include "common/common.h"
#include "physics/physics.h"
#include "gazebo.hh"

namespace gazebo
{
  class GripperPlugin : public ModelPlugin
  {
    public: GripperPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();
    private: void OnContact(const std::string &_collisionName,
                            const physics::Contact &_contact);

    private: void HandleAttach();
    private: void HandleDetach();
    private: void ResetDiffs();

    private: physics::ModelPtr model;
    private: physics::PhysicsEnginePtr physics;
    private: physics::JointPtr fixedJoint;

    private: std::vector<physics::JointPtr> joints;
    private: std::vector<physics::BodyPtr> links;
    private: std::vector<event::ConnectionPtr> connections;

    private: std::map<std::string, physics::CollisionPtr> collisions;
    private: std::vector<physics::Contact> contacts;

    private: bool attached;

    private: math::Pose prevDiff;
    private: std::vector<double> diffs;
    private: int diffIndex;

    private: common::Time updateRate, prevUpdateTime;
    private: int posCount, zeroCount;
  };
}
#endif
