/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
#include "Body.hh"
#include "SensorFactory.hh"

using namespace gazebo;

std::map<std::string, SensorFactoryFn> SensorFactory::sensors = std::map<std::string, SensorFactoryFn>();//std::make_pair(std::string("test"),SensorFactoryFn()));

// Register a model class.  Use by dynamically loaded modules
void SensorFactory::RegisterSensor(std::string type, std::string classname,
                                   SensorFactoryFn factoryfn)
{
  if (sensors.size() == 0)  // bug in gcc 4.6 initialization of static map
    sensors = std::map<std::string, SensorFactoryFn>();
  sensors[classname] = factoryfn;
}


// Create a new instance of a model.  Used by the world when reading
// the world file.
Sensor *SensorFactory::NewSensor(const std::string &classname, Body *body)
{
  if (sensors[classname])
  {
    Sensor *sensor = (sensors[classname]) (body);
    SensorManager::Instance()->AddSensor(sensor);
    return sensor;
  }

  return NULL;
}
