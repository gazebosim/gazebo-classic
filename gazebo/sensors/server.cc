/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <signal.h>
#include <iostream>
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderEngine.hh"

#include "gazebo/gazebo.hh"

bool quit = false;

//////////////////////////////////////////////////
void SignalHandler(int)
{
  quit = true;
}

void Load()
{
  gazebo::load();

  /// Init the sensors library
  gazebo::sensors::load();
  gazebo::sensors::init();

  gazebo::rendering::create_scene("world_1", false, true);
  gazebo::common::Time::MSleep(10);
}

void Run()
{
  gazebo::run();
  while (!quit)
  {
    gazebo::sensors::run_once(true);
    gazebo::common::Time::MSleep(10);
  }
}

int main(int /*argc*/, char ** /*argv*/)
{
  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  Load();
  Run();

  return 0;
}
