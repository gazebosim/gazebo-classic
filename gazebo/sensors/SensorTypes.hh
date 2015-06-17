/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/util/system.hh"

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
    class MagnetometerSensor;
    class OrientationSensor;
    class FluidPressureSensor;
    class GpuRaySensor;
    class RFIDSensor;
    class RFIDTag;
    class SonarSensor;
    class ForceTorqueSensor;
    class GpsSensor;
    class WirelessTransceiver;
    class WirelessTransmitter;
    class WirelessReceiver;

    class Noise;
    class ExponentialNoiseModel;
    class GaussianNoiseModel;
    class GPMapNoiseModel;
    class ImageGaussianNoiseModel;
    class OrnsteinNoiseModel;
    class WienerNoiseModel;

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

    /// \def ForceTorqueSensorPtr
    /// \brief Shared pointer to ForceTorqueSensor
    typedef boost::shared_ptr<ForceTorqueSensor> ForceTorqueSensorPtr;

    /// \def FLuidPressureSensorPtr
    /// \brief Shared pointer to FluidPressureSensor
    typedef boost::shared_ptr<FluidPressureSensor> FluidPressureSensorPtr;

    /// \def GpsSensorPtr
    /// \brief Shared pointer to GpsSensor
    typedef boost::shared_ptr<GpsSensor> GpsSensorPtr;

    /// \def GpuRaySensorPtr
    /// \brief Shared pointer to GpuRaySensor
    typedef boost::shared_ptr<GpuRaySensor> GpuRaySensorPtr;

    /// \def ImuSensorPtr
    /// \brief Shared pointer to ImuSensor
    typedef boost::shared_ptr<ImuSensor> ImuSensorPtr;

    /// \def MagnetometerSensorPtr
    /// \brief Shared pointer to ImuSensor
    typedef boost::shared_ptr<MagnetometerSensor> MagnetometerSensorPtr;

    /// \def RFIDSensorPtr
    /// \brief Shared pointer to RFIDSensor
    typedef boost::shared_ptr<RFIDSensor> RFIDSensorPtr;

    /// \def RFIDTagPtr
    /// \brief Shared pointer to RFIDTag
    typedef boost::shared_ptr<RFIDTag> RFIDTagPtr;

    /// \def SonarSensorPtr
    /// \brief Shared pointer to SonarSensor
    typedef boost::shared_ptr<SonarSensor> SonarSensorPtr;

    /// \def OrientationSensorPtr
    /// \brief Shared pointer to OrientationSensor
    typedef boost::shared_ptr<OrientationSensor> OrientationSensorPtr;

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

    /// \def FluidPressureSensor_V
    /// \brief Vector of FluidPRessureSensor shared pointers
    typedef std::vector<FluidPressureSensorPtr> FluidPressureSensor_V;

    /// \def GpuRaySensor_V
    /// \brief Vector of GpuRaySensor shared pointers
    typedef std::vector<GpuRaySensorPtr> GpuRaySensor_V;

    /// \def GpsSensor_V
    /// \brief Vector of GpsSensor shared pointers
    typedef std::vector<GpsSensorPtr> GpsSensor_V;

    /// \def ImuSensor_V
    /// \brief Vector of ImuSensor shared pointers
    typedef std::vector<ImuSensorPtr> ImuSensor_V;

    /// \def MagnetometerSensor_V
    /// \brief Vector of MagnetometerSensor shared pointers
    typedef std::vector<MagnetometerSensorPtr> MagnetometerSensor_V;

    /// \def OrientationSensor_V
    /// \brief Vector of OrientationSensor shared pointers
    typedef std::vector<OrientationSensorPtr> OrientationSensor_V;

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

    /// \def NoisePtr
    /// \brief Shared pointer to Noise
    typedef boost::shared_ptr<Noise> NoisePtr;

    /// \def ExponentialNoisePtr
    /// \brief Shared pointer to exponential noise model
    typedef boost::shared_ptr<ExponentialNoiseModel> ExponentialNoiseModelPtr;

    /// \def GaussianNoisePtr
    /// \brief Shared pointer to gaussian noise model
    typedef boost::shared_ptr<GaussianNoiseModel> GaussianNoiseModelPtr;

    /// \def GaussianNoisePtr
    /// \brief Shared pointer to image gaussian noise model
    typedef boost::shared_ptr<ImageGaussianNoiseModel> ImageGaussianNoiseModelPtr;

    /// \def GPMaplNoisePtr
    /// \brief Shared pointer to GP map noise model
    typedef boost::shared_ptr<GPMapNoiseModel> GPMapNoiseModelPtr;

    /// \def WienerNoisePtr
    /// \brief Shared pointer to wiener noise model
    typedef boost::shared_ptr<WienerNoiseModel> WienerNoiseModelPtr;

    /// \def OrnsteinNoisePtr
    /// \brief Shared pointer to ornstein noise model
    typedef boost::shared_ptr<OrnsteinNoiseModel> OrnsteinNoiseModelPtr;

    /// \def SensorNoiseTypes
    /// \brief Eumeration of all sensor noise types
    enum SensorNoiseTypes
    {
      /// \brief Noise streams for the Camera sensor
      /// \sa CameraSensor
      CameraNoise = 0,

      /// \brief Noise streams for the fluid pressure sensor
      /// \sa FluidPressureSensor
      FluidPressureNoise = 1,

      /// \brief Noise streams for the GPS sensor
      /// \sa GpsSensor
      PosLatNoiseMeters = 2,
      PosLonNoiseMeters = 3, 
      PosAltNoiseMeters = 4,
      VelLatNoiseMeters = 5, 
      VelLonNoiseMeters = 6,
      VelAltNoiseMeters = 7,

      /// \brief Noise streams for the inertial measurement unit sensor
      /// \sa ImuSensor
      AngVelNoiseX = 8,
      AngVelNoiseY = 9,
      AngVelNoiseZ = 10,
      LinAccNoiseX = 11,
      LinAccNoiseY = 12, 
      LinAccNoiseZ = 13,

      /// \brief Noise streams for the magnetometer sensor
      /// \sa MagnetometerSensor
      MagneticFieldNoiseX = 14,
      MagneticFieldNoiseY = 15,
      MagneticFieldNoiseZ = 16,

      /// \brief Noise streams for the Orientation sensor
      /// \sa MagnetometerSensor
      OrientationNoiseX = 17,
      OrientationNoiseY = 18,
      OrientationNoiseZ = 19,

      /// \brief Noise streams for the multicamera sensor
      /// \sa MultiCameraSensor
      MultiCameraNoise = 20

    };

    /// \}
  }
}
#endif
