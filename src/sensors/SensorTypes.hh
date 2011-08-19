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
#ifndef SENSORTYPES_HH
#define SENSORTYPES_HH

#include <vector>
#include <boost/shared_ptr.hpp>

/// \file
/// \ingroup gazebo_sensors
/// \brief Forward declarations and typedefs for sensors
namespace gazebo
{
  namespace sensors
  {
    class Sensor;
    class RaySensor;
    class ContactSensor;

    typedef boost::shared_ptr<Sensor> SensorPtr;
    typedef boost::shared_ptr<RaySensor> RaySensorPtr;
    typedef boost::shared_ptr<ContactSensor> ContactSensorPtr;
    typedef std::vector<SensorPtr> Sensor_V;
    typedef std::vector<RaySensorPtr> RaySensor_V;
    typedef std::vector<ContactSensorPtr> ContactSensor_V;
  }
}
 
#endif
