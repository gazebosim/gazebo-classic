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
/*
 * Desc: Class to manager all sensors
 * Author: Nate Koenig
 * Date: 18 Dec 2009
 */

#include "rendering/RenderEngine.hh"
#include "sensors/Sensor.hh"
#include "sensors/SensorManager.hh"

using namespace gazebo;
using namespace sensors;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
SensorManager::SensorManager()
  : stop(false), runThread(NULL)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
SensorManager::~SensorManager()
{
  this->sensors.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Run the sensor manager update in a new thread
void SensorManager::Run()
{
  this->runThread = new boost::thread( 
      boost::bind(&SensorManager::RunLoop, this));

  gzdbg << "Created thread[" << this->runThread->get_id() << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
// Stop the run thread
void SensorManager::Stop()
{
  this->stop = true;
  if (this->runThread)
  {
    gzdbg << "Created thread[" << this->runThread->get_id() << "]\n";
    this->runThread->join();
    delete this->runThread;
    this->runThread = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update loop
void SensorManager::RunLoop()
{
  while (!this->stop)
    this->Update();
}

////////////////////////////////////////////////////////////////////////////////
/// Update all the sensors
void SensorManager::Update(bool force)
{
  event::Events::preRender();

  // Tell all the cameras to render
  event::Events::render();

  event::Events::postRender();

  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
  {
    //gzerr << "SensorManager Update [" << (*iter)->GetName() << "]\n";
    (*iter)->Update(force);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Init all the sensors
void SensorManager::Init()
{
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->Init();
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize all the sensors
void SensorManager::Fini()
{
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Add a sensor
void SensorManager::AddSensor(SensorPtr sensor)
{
  this->sensors.push_back(sensor);
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a sensor
void SensorManager::RemoveSensor(SensorPtr sensor)
{
  std::list<SensorPtr>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    if ((*iter)->GetName() == sensor->GetName())
      break;

  if (iter != this->sensors.end())
  {
    this->sensors.erase(iter);
  }
}
