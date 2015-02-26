/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class Issue1208Plugin : public WorldPlugin
  {
    public: Issue1208Plugin() : WorldPlugin(), initCount(0)
            {
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
            {
              this->world = _world;
              this->physics = this->world->GetPhysicsEngine();
            }

    public: void Init()
            {
              ++initCount;
              physics->SetRealTimeUpdateRate(initCount);
            }

    private: physics::WorldPtr world;
    private: physics::PhysicsEnginePtr physics;
    private: int initCount;
  };
  GZ_REGISTER_WORLD_PLUGIN(Issue1208Plugin)
}
