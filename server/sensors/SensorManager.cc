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
 * Desc: Class to manager all sensors
 * Author: Nate Koenig
 * Date: 18 Dec 2009
 * SVN info: $Id$
 */

#include "Sensor.hh"
#include "SensorManager.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
SensorManager::SensorManager()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
SensorManager::~SensorManager()
{
  this->sensors.erase(this->sensors.begin(), this->sensors.end());
}

////////////////////////////////////////////////////////////////////////////////
/// Update all the sensors
void SensorManager::Update()
{
  std::list<Sensor*>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->Update();
    
}

////////////////////////////////////////////////////////////////////////////////
/// Init all the sensors
void SensorManager::Init()
{
  std::list<Sensor*>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->Init();
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize all the sensors
void SensorManager::Fini()
{
  std::list<Sensor*>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    (*iter)->Fini();
}


////////////////////////////////////////////////////////////////////////////////
/// Add a sensor
void SensorManager::AddSensor(Sensor *sensor)
{
  if (!sensor)
    return;

  this->sensors.push_back(sensor);
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a sensor
void SensorManager::RemoveSensor(Sensor *sensor)
{
  if (!sensor)
    return;

  std::list<Sensor*>::iterator iter;
  for (iter = this->sensors.begin(); iter != this->sensors.end(); iter++)
    if (*iter == sensor)
      break;

  if (iter != this->sensors.end())
  {
    delete (*iter);
    this->sensors.erase(iter);
  }
}
