/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_SIMULATIONPRIVATE_HH_
#define GAZEBO_PLUGINS_SIMULATIONPRIVATE_HH_

#include <ignition/transport.hh>

namespace gazebo
{

class DistributedSimulationPrivate
{
  public: std::unique_ptr<ignition::transport::Node> node;
  public: physics::WorldPtr world;
};

}

#endif

