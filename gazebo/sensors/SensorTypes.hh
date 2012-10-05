/*
 * Copyright 2011 Nate Koenig
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
    class CameraSensor;
    class DepthCameraSensor;
    class ContactSensor;
    class GpuRaySensor;
    class RFIDSensor;
    class RFIDTag;

    typedef boost::shared_ptr<Sensor> SensorPtr;
    typedef boost::shared_ptr<RaySensor> RaySensorPtr;
    typedef boost::shared_ptr<CameraSensor> CameraSensorPtr;
    typedef boost::shared_ptr<DepthCameraSensor> DepthCameraSensorPtr;
    typedef boost::shared_ptr<ContactSensor> ContactSensorPtr;
    typedef boost::shared_ptr<GpuRaySensor> GpuRaySensorPtr;
    typedef boost::shared_ptr<RFIDSensor> RFIDSensorPtr;
    typedef boost::shared_ptr<RFIDTag> RFIDTagPtr;

    typedef std::vector<SensorPtr> Sensor_V;
    typedef std::vector<RaySensorPtr> RaySensor_V;
    typedef std::vector<CameraSensorPtr> CameraSensor_V;
    typedef std::vector<DepthCameraSensorPtr> DepthCameraSensor_V;
    typedef std::vector<ContactSensorPtr> ContactSensor_V;
    typedef std::vector<GpuRaySensorPtr> GpuRaySensor_V;
    typedef std::vector<RFIDSensor> RFIDSensor_V;
    typedef std::vector<RFIDTag> RFIDTag_V;
  }
}
#endif
