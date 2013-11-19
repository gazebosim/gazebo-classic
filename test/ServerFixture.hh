/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SERVER_FIXTURE_HH_
#define _GAZEBO_SERVER_FIXTURE_HH_

#pragma GCC diagnostic ignored "-Wswitch-default"
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wshadow"

// The following is needed to enable the GetMemInfo function for OSX
#ifdef __MACH__
# include <mach/mach.h>
#endif  // __MACH__

#include <sdf/sdf.hh>

#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include <map>
#include <string>

#include "gazebo/transport/transport.hh"

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/gazebo_config.h"
#include "gazebo/Server.hh"

#include "test_config.h"

using namespace gazebo;

std::string custom_exec(std::string _cmd);

class ServerFixture : public testing::Test
{
  protected: ServerFixture();

  protected: virtual void TearDown();

  protected: virtual void Unload();

  protected: virtual void Load(const std::string &_worldFilename);

  protected: virtual void Load(const std::string &_worldFilename, bool _paused);

  protected: virtual void Load(const std::string &_worldFilename,
                               bool _paused, const std::string &_physics);

  protected: void RunServer(const std::string &_worldFilename);

  protected: rendering::ScenePtr GetScene(
                 const std::string &_sceneName = "default");

  protected: void RunServer(const std::string &_worldFilename, bool _paused,
                            const std::string &_physics);

  protected: void OnStats(ConstWorldStatisticsPtr &_msg);

  protected: void SetPause(bool _pause);

  protected: double GetPercentRealTime() const;

  protected: void OnPose(ConstPosesStampedPtr &_msg);

  protected: math::Pose GetEntityPose(const std::string &_name);

  protected: bool HasEntity(const std::string &_name);

  protected: void PrintImage(const std::string &_name, unsigned char **_image,
                unsigned int _width, unsigned int _height, unsigned int _depth);

  protected: void PrintScan(const std::string &_name, double *_scan,
                            unsigned int _sampleCount);

  protected: void FloatCompare(float *_scanA, float *_scanB,
                 unsigned int _sampleCount, float &_diffMax,
                 float &_diffSum, float &_diffAvg);

  protected: void DoubleCompare(double *_scanA, double *_scanB,
                 unsigned int _sampleCount, double &_diffMax,
                 double &_diffSum, double &_diffAvg);

  protected: void ImageCompare(unsigned char *_imageA,
                 unsigned char *_imageB,
                 unsigned int _width, unsigned int _height, unsigned int _depth,
                 unsigned int &_diffMax, unsigned int &_diffSum,
                 double &_diffAvg);

  private: void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth,
                              const std::string &/*_format*/);

  protected: void GetFrame(const std::string &_cameraName,
                 unsigned char **_imgData, unsigned int &_width,
                 unsigned int &_height);

  protected: void SpawnCamera(const std::string &_modelName,
                 const std::string &_cameraName,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 unsigned int _width = 320, unsigned int _height = 240,
                 double _rate = 25,
                 const std::string &_noiseType = "",
                 double _noiseMean = 0.0,
                 double _noiseStdDev = 0.0);

  protected: void SpawnRaySensor(const std::string &_modelName,
                 const std::string &_raySensorName,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 double _hMinAngle = -2.0, double _hMaxAngle = 2.0,
                 double _minRange = 0.08, double _maxRange = 10,
                 double _rangeResolution = 0.01, unsigned int _samples = 640,
                 const std::string &_noiseType = "", double _noiseMean = 0.0,
                 double _noiseStdDev = 0.0);

  protected: void SpawnGpuRaySensor(const std::string &_modelName,
                 const std::string &_raySensorName,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 double _hMinAngle = -2.0, double _hMaxAngle = 2.0,
                 double _minRange = 0.08, double _maxRange = 10,
                 double _rangeResolution = 0.01, unsigned int _samples = 640,
                 const std::string &_noiseType = "", double _noiseMean = 0.0,
                 double _noiseStdDev = 0.0);

  protected: void SpawnImuSensor(const std::string &_modelName,
                 const std::string &_imuSensorName,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 const std::string &_noiseType = "",
                 double _rateNoiseMean = 0.0, double _rateNoiseStdDev = 0.0,
                 double _rateBiasMean = 0.0, double _rateBiasStdDev = 0.0,
                 double _accelNoiseMean = 0.0, double _accelNoiseStdDev = 0.0,
                 double _accelBiasMean = 0.0, double _accelBiasStdDev = 0.0);

  /// \brief Spawn a contact sensor with the specified collision geometry
  /// \param[in] _name Model name
  /// \param[in] _sensorName Sensor name
  /// \param[in] _collisionType Type of collision, box or cylinder
  /// \param[in] _pos World position
  /// \param[in] _rpy World rotation in Euler angles
  /// \param[in] _static True to make the model static
  protected: void SpawnUnitContactSensor(const std::string &_name,
                 const std::string &_sensorName,
                 const std::string &_collisionType, const math::Vector3 &_pos,
                 const math::Vector3 &_rpy, bool _static = false);

  /// \brief Spawn an IMU sensor on a link
  /// \param[in] _name Model name
  /// \param[in] _sensorName Sensor name
  /// \param[in] _collisionType Type of collision, box or cylinder
  /// \param[in] _topic Topic to publish IMU data to
  /// \param[in] _pos World position
  /// \param[in] _rpy World rotation in Euler angles
  /// \param[in] _static True to make the model static
  protected: void SpawnUnitImuSensor(const std::string &_name,
                 const std::string &_sensorName,
                 const std::string &_collisionType,
                 const std::string &_topic, const math::Vector3 &_pos,
                 const math::Vector3 &_rpy, bool _static = false);

  /// \brief generate a gtest failure from a timeout error and display a
  /// log message about the problem.
  /// \param[in] log_msg: error msg related to the timeout
  /// \param[in] timeoutCS: failing period (in centiseconds)
  private: void launchTimeoutFailure(const char *_logMsg, const int _timeoutCS);

  /// \brief Spawn an Wireless transmitter sensor on a link
  /// \param[in] _name Model name
  /// \param[in] _sensorName Sensor name
  /// \param[in] _pos World position
  /// \param[in] _rpy World rotation in Euler angles
  /// \param[in] _essid Service set identifier (network name)
  /// \param[in] _freq Frequency of transmission (MHz)
  /// \param[in] _power Transmission power (dBm)
  /// \param[in] _gain Antenna gain (dBi)
  /// \param[in] _visualize Enable sensor visualization
  protected: void SpawnWirelessTransmitterSensor(const std::string &_name,
                 const std::string &_sensorName,
                 const math::Vector3 &_pos,
                 const math::Vector3 &_rpy,
                 const std::string &_essid,
                 double _freq,
                 double _power,
                 double _gain,
                 bool _visualize = true);

  /// \brief Spawn an Wireless receiver sensor on a link
  /// \param[in] _name Model name
  /// \param[in] _sensorName Sensor name
  /// \param[in] _pos World position
  /// \param[in] _rpy World rotation in Euler angles
  /// \param[in] _minFreq Minimum frequency to be filtered (MHz)
  /// \param[in] _maxFreq Maximum frequency to be filtered (MHz)
  /// \param[in] _power Transmission power (dBm)
  /// \param[in] _gain Antenna gain (dBi)
  /// \param[in] _sensitivity Receiver sensitibity (dBm)
  /// \param[in] _visualize Enable sensor visualization
  protected: void SpawnWirelessReceiverSensor(const std::string &_name,
                 const std::string &_sensorName,
                 const math::Vector3 &_pos,
                 const math::Vector3 &_rpy,
                 double _minFreq,
                 double _maxFreq,
                 double _power,
                 double _gain,
                 double _sensitivity,
                 bool _visualize = true);

  /// \brief Wait for a number of ms. and attempts until the entity is spawned
  /// \param[in] _name Model name
  /// \param[in] _sleepEach Number of milliseconds to sleep in each iteration
  /// \param[in] _retries Number of iterations until give up
  private: void WaitUntilEntitySpawn(const std::string &_name,
                                     unsigned int _sleepEach,
                                     int _retries);

  /// \brief Wait for a number of ms. and attempts until the sensor is spawned
  /// \param[in] _name Sensor name
  /// \param[in] _sleepEach Number of milliseconds to sleep in each iteration
  /// \param[in] _retries Number of iterations until give up
  private: void WaitUntilSensorSpawn(const std::string &_name,
                                     unsigned int _sleepEach,
                                     int _retries);

  protected: void SpawnCylinder(const std::string &_name,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 bool _static = false);

  protected: void SpawnSphere(const std::string &_name,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 bool _wait = true, bool _static = false);

  protected: void SpawnSphere(const std::string &_name,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 const math::Vector3 &_cog, double _radius,
                 bool _wait = true, bool _static = false);

  protected: void SpawnBox(const std::string &_name,
                 const math::Vector3 &_size, const math::Vector3 &_pos,
                 const math::Vector3 &_rpy, bool _static = false);

  protected: void SpawnTrimesh(const std::string &_name,
                 const std::string &_modelPath, const math::Vector3 &_scale,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 bool _static = false);

  protected: void SpawnEmptyLink(const std::string &_name,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy,
                 bool _static = false);

  protected: void SpawnModel(const std::string &_filename);

             /// \brief Send a factory message based on an SDF string.
             /// \param[in] _sdf SDF string to publish.
  protected: void SpawnSDF(const std::string &_sdf);

  protected: void LoadPlugin(const std::string &_filename,
                             const std::string &_name);

  protected: physics::ModelPtr GetModel(const std::string &_name);

  protected: void RemoveModel(const std::string &_name);

  protected: void RemovePlugin(const std::string &_name);

  protected: void GetMemInfo(double &_resident, double &_share);

  protected: Server *server;
  protected: boost::thread *serverThread;

  protected: transport::NodePtr node;
  protected: transport::SubscriberPtr poseSub;
  protected: transport::SubscriberPtr statsSub;
  protected: transport::PublisherPtr factoryPub;
  protected: transport::PublisherPtr requestPub;

  protected: std::map<std::string, math::Pose> poses;
  protected: boost::mutex receiveMutex;

  private: unsigned char **imgData;
  private: int gotImage;

  protected: common::Time simTime, realTime, pauseTime;
  private: double percentRealTime;
  private: bool paused;
  private: bool serverRunning;
};
#endif  // define _GAZEBO_SERVER_FIXTURE_HH_
