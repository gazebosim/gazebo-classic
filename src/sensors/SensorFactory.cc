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
 * Desc: Factory for creating sensor
 * Author: Andrew Howard
 * Date: 18 May 2003
 * SVN info: $Id$
 */

#include "Sensor.hh"
#include "SensorManager.hh"
#include "SensorFactory.hh"

using namespace gazebo;

std::map<std::string, SensorFactoryFn> SensorFactory::sensors;


// Register a model class.  Use by dynamically loaded modules
void SensorFactory::RegisterSensor(std::string type, std::string classname,
                                   SensorFactoryFn factoryfn)
{
  sensors[classname] = factoryfn;
}


// Create a new instance of a model.  Used by the world when reading
// the world file.
Sensor *SensorFactory::NewSensor(const std::string &classname)
{
  if (sensors[classname])
  {
    Sensor *sensor = (sensors[classname]) ();
    SensorManager::Instance()->AddSensor(sensor);
    return sensor;
  }

  return NULL;
}
