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
#ifndef _SENSORTYPES_HH_
#define _SENSORTYPES_HH_

#include <vector>
#include <boost/shared_ptr.hpp>

/// \file
/// \ingroup gazebo_sensors
/// \brief Forward declarations and typedefs for sensors
namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    class Sensor;
    class RaySensor;
    class CameraSensor;
    class MultiCameraSensor;
    class DepthCameraSensor;
    class ContactSensor;
    class ImuSensor;
    class GpuRaySensor;
    class RFIDSensor;
    class RFIDTag;
    class SonarSensor;
    class ForceTorqueSensor;
    class GpsSensor;
    class Noise;
    class WirelessTransceiver;
    class WirelessTransmitter;
    class WirelessReceiver;

    /// \def SensorPtr
    /// \brief Shared pointer to Sensor
    typedef boost::shared_ptr<Sensor> SensorPtr;

    /// \def RaySensorPtr
    /// \brief Shared pointer to RaySensor
    typedef boost::shared_ptr<RaySensor> RaySensorPtr;

    /// \def CameraSensorPtr
    /// \brief Shared pointer to CameraSensor
    typedef boost::shared_ptr<CameraSensor> CameraSensorPtr;

    /// \def MultiCameraSensorPtr
    /// \brief Shared pointer to MultiCameraSensor
    typedef boost::shared_ptr<MultiCameraSensor> MultiCameraSensorPtr;

    /// \def DepthCameraSensorPtr
    /// \brief Shared pointer to DepthCameraSensor
    typedef boost::shared_ptr<DepthCameraSensor> DepthCameraSensorPtr;

    /// \def ContactSensorPtr
    /// \brief Shared pointer to ContactSensor
    typedef boost::shared_ptr<ContactSensor> ContactSensorPtr;

    /// \def ImuSensorPtr
    /// \brief Shared pointer to ImuSensor
    typedef boost::shared_ptr<ImuSensor> ImuSensorPtr;

    /// \def GpuRaySensorPtr
    /// \brief Shared pointer to GpuRaySensor
    typedef boost::shared_ptr<GpuRaySensor> GpuRaySensorPtr;

    /// \def RFIDSensorPtr
    /// \brief Shared pointer to RFIDSensor
    typedef boost::shared_ptr<RFIDSensor> RFIDSensorPtr;

    /// \def RFIDTagPtr
    /// \brief Shared pointer to RFIDTag
    typedef boost::shared_ptr<RFIDTag> RFIDTagPtr;

    /// \def SonarSensorPtr
    /// \brief Shared pointer to SonarSensor
    typedef boost::shared_ptr<SonarSensor> SonarSensorPtr;

    /// \def ForceTorqueSensorPtr
    /// \brief Shared pointer to ForceTorqueSensor
    typedef boost::shared_ptr<ForceTorqueSensor> ForceTorqueSensorPtr;

    /// \def GpsSensorPtr
    /// \brief Shared pointer to GpsSensor
    typedef boost::shared_ptr<GpsSensor> GpsSensorPtr;

    /// \def NoisePtr
    /// \brief Shared pointer to Noise
    typedef boost::shared_ptr<Noise> NoisePtr;

    /// \def WirelessTransceiverPtr
    /// \brief Shared pointer to WirelessTransceiver
    typedef boost::shared_ptr<WirelessTransceiver> WirelessTransceiverPtr;

    /// \def WirelessTransmitterPtr
    /// \brief Shared pointer to WirelessTransmitter
    typedef boost::shared_ptr<WirelessTransmitter> WirelessTransmitterPtr;

    /// \def WirelessReceiverPtr
    /// \brief Shared pointer to WirelessReceiver
    typedef boost::shared_ptr<WirelessReceiver> WirelessReceiverPtr;

    /// \def Sensor_V
    /// \brief Vector of Sensor shared pointers
    typedef std::vector<SensorPtr> Sensor_V;

    /// \def RaySensor_V
    /// \brief Vector of RaySensor shared pointers
    typedef std::vector<RaySensorPtr> RaySensor_V;

    /// \def CameraSensor_V
    /// \brief Vector of CameraSensor shared pointers
    typedef std::vector<CameraSensorPtr> CameraSensor_V;

    /// \def MultiCameraSensor_V
    /// \brief Vector of MultiCameraSensor shared pointers
    typedef std::vector<MultiCameraSensorPtr> MultiCameraSensor_V;

    /// \def DepthCameraSensor_V
    /// \brief Vector of DepthCameraSensor shared pointers
    typedef std::vector<DepthCameraSensorPtr> DepthCameraSensor_V;

    /// \def ContactSensor_V
    /// \brief Vector of ContactSensor shared pointers
    typedef std::vector<ContactSensorPtr> ContactSensor_V;

    /// \def ImuSensor_V
    /// \brief Vector of ImuSensor shared pointers
    typedef std::vector<ImuSensorPtr> ImuSensor_V;

    /// \def GpuRaySensor_V
    /// \brief Vector of GpuRaySensor shared pointers
    typedef std::vector<GpuRaySensorPtr> GpuRaySensor_V;

    /// \def RFIDSensor_V
    /// \brief Vector of RFIDSensors
    typedef std::vector<RFIDSensor> RFIDSensor_V;

    /// \def RFIDTag_V
    /// \brief Vector of RFIDTags
    typedef std::vector<RFIDTag> RFIDTag_V;

    /// \def WirelessTransceiver_V
    /// \brief Vector of WirelessTransceiver
    typedef std::vector<WirelessTransceiver> WirelessTransceiver_V;

    /// \def WirelessTransmitter_V
    /// \brief Vector of WirelessTransmitter
    typedef std::vector<WirelessTransmitter> WirelessTransmitter_V;

    /// \def WirelessReceiver_V
    /// \brief Vector of WirelessReceiver
    typedef std::vector<WirelessReceiver> WirelessReceiver_V;


    /// \}
  }
}
#endif
