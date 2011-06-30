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
#include "common/Timer.hh"
#include "common/SystemPaths.hh"
#include "transport/Transport.hh"

#include "sensors/Sensors.hh"

#include "sdf/interface/sdf.h"
#include "rendering/Rendering.hh"

#include "SensorServer.hh"

using namespace gazebo;

SensorServer::SensorServer()
{
  this->quit = false;

  // load the configuration options 
  try
  {
    common::SystemPaths::Instance()->Load();
  }
  catch (common::Exception e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  // Start the transport system by connecting to the master.
  transport::init();

  /// Init the sensors library
  sensors::init("default");
}

SensorServer::~SensorServer()
{
}

void SensorServer::Load(const std::string &/*filename*/)
{
  // Load the rendering system
  if (!rendering::load())
    gzthrow("Unable to load the rendering engine");

  // The rendering engine will run headless 
  if (!rendering::init())
    gzthrow("Unable to intialize the rendering engine");

  rendering::create_scene("default");

  sensors::SensorPtr sensor1 = sensors::create_sensor("camera");
  sensors::SensorPtr sensor2 = sensors::create_sensor("camera");
}

void SensorServer::Run()
{
  common::Timer timer;
  while (!this->quit)
  {
    timer.Start();
    sensors::run_once(true);
  }
}

void SensorServer::Quit()
{
  this->quit = true;
}
