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
#include <vector>

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

#include "gazebo/math/SignalStats.hh"
#include "gazebo/math/Vector3Stats.hh"

#include "gazebo/gazebo_config.h"
#include "gazebo/Server.hh"
#include "gazebo/util/system.hh"

#include "test_config.h"

namespace gazebo
{
  std::string custom_exec(std::string _cmd);

  class GAZEBO_VISIBLE ServerFixture : public testing::Test
  {
    /// \brief Constructor
    protected: ServerFixture();

    /// \brief Destructor
    protected: virtual ~ServerFixture();

    /// \brief Tear down the test fixture. This gets called by gtest.
    protected: virtual void TearDown();

    /// \brief Unload the test fixture.
    protected: virtual void Unload();

    /// \brief Load a world based on a filename.
    /// \param[in] _worldFilename Name of the world to load.
    protected: virtual void Load(const std::string &_worldFilename);

    /// \brief Load a world based on a filename and set simulation
    /// paused/un-paused.
    /// \param[in] _worldFilename Name of the world to load.
    /// \param[in] _paused True to start the world paused.
    protected: virtual void Load(const std::string &_worldFilename,
                                 bool _paused);

    /// \brief Load a world based on a filename and set simulation
    /// paused/un-paused, and specify physics engine. This versions allows
    /// plugins to be loaded (via the cmd line args)
    /// \param[in] _worldFilename Name of the world to load.
    /// \param[in] _paused True to start the world paused.
    /// \param[in] _physics Name of the physics engine.
    /// \param[in] _systemPlugins Array of system plugins to load.
    protected: virtual void Load(const std::string &_worldFilename,
                                 bool _paused, const std::string &_physics,
                          const std::vector<std::string> &_systemPlugins = {});

     /// \brief
    protected: virtual void LoadArgs(const std::string &_args,
                          const std::vector<std::string> &_systemPlugins = {});

    /// \brief Run the server.
    /// \param[in] _worldFilename Name of the world to run in simulation.
    protected: void RunServer(const std::string &_worldFilename);

    /// \brief Run the server, start paused/unpaused, and specify the physics
    /// engine.
    /// \param[in] _worldFilename Name of the world to load.
    /// \param[in] _paused True to start the world paused.
    /// \param[in] _physics Name of the physics engine.
    /// \param[in] _systemPlugins Array of system plugins to load.
    protected: void RunServer(const std::string &_worldFilename, bool _paused,
                              const std::string &_physics,
                          const std::vector<std::string> &_systemPlugins = {});

    /// \brief
    protected: void RunServerArgs(const std::string &_args,
                          const std::vector<std::string> &_systemPlugins = {});


    /// \brief Get a pointer to the rendering scene.
    /// \param[in] _sceneName Name of the scene to get.
    protected: rendering::ScenePtr GetScene(
                   const std::string &_sceneName = "default");


    /// \brief Function that received world stastics messages.
    /// \param[in] _msg World statistics message.
    protected: void OnStats(ConstWorldStatisticsPtr &_msg);

    /// \brief Set a running simulation paused/unpaused.
    protected: void SetPause(bool _pause);

    /// \brief Get the real-time factor.
    /// \return The percent real time simulation is running.
    protected: double GetPercentRealTime() const;

    /// \brief Function that received poses messages from a running
    /// simulation.
    /// \param[in] _msg Pose message.
    protected: void OnPose(ConstPosesStampedPtr &_msg);

    /// \brief Get the pose of an entity.
    /// \param[in] _name Name of the entity.
    /// \return Pose of the named entity.
    protected: math::Pose GetEntityPose(const std::string &_name);

    /// \brief Return true if the named entity exists.
    /// \param[in] _name Name of the entity to check for.
    /// \return True if the entity exists.
    protected: bool HasEntity(const std::string &_name);

    /// \brief Print image data to screen. This is used to generate test data.
    /// \param[in] _name Name to associate with the printed data.
    /// \param[in] _image The raw image data.
    /// \param[in] _width Width of the image.
    /// \param[in] _height Height of the image.
    /// \param[in] _depth Pixel depth.
    protected: void PrintImage(const std::string &_name, unsigned char **_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth);

    /// \brief Print laser scan to screen. This is used to generate test data.
    /// \param[in] _name Name to associate with the printed data.
    /// \param[in] _scan The laser scan data.
    /// \param[in] _sampleCount Number of samples in the scan data.
    protected: void PrintScan(const std::string &_name, double *_scan,
                              unsigned int _sampleCount);

    /// \brief Function to compare two float arrays (for example two laser
    /// scans).
    /// \param[in] _scanA First float array.
    /// \param[in] _scanB Second float array.
    /// \param[in] _sampleCount Number of samples in each float array.
    /// \param[out] _diffMax Maximum difference between the two arrays.
    /// \param[out] _diffSum Sum of the differences between the two arrays.
    /// \param[out] _diffAvg Average difference between the two arrays.
    protected: void FloatCompare(float *_scanA, float *_scanB,
                   unsigned int _sampleCount, float &_diffMax,
                   float &_diffSum, float &_diffAvg);

    /// \brief Function to compare two double arrays (for example two laser
    /// scans).
    /// \param[in] _scanA First double array.
    /// \param[in] _scanB Second double array.
    /// \param[in] _sampleCount Number of samples in each double array.
    /// \param[out] _diffMax Maximum difference between the two arrays.
    /// \param[out] _diffSum Sum of the differences between the two arrays.
    /// \param[out] _diffAvg Average difference between the two arrays.
    protected: void DoubleCompare(double *_scanA, double *_scanB,
                   unsigned int _sampleCount, double &_diffMax,
                   double &_diffSum, double &_diffAvg);

    /// \brief Function to compare two images.
    /// \param[in] _imageA First image to compare.
    /// \param[in] _imageB Second image to compare.
    /// \param[in] _width Width of both images.
    /// \param[in] _height Height of both images.
    /// \param[in] _depth Depth of both images.
    /// \param[out] _diffMax Maximum difference between the two arrays.
    /// \param[out] _diffSum Sum of the differences between the two arrays.
    /// \param[out] _diffAvg Average difference between the two arrays.
    protected: void ImageCompare(unsigned char *_imageA,
                   unsigned char *_imageB,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth,
                   unsigned int &_diffMax, unsigned int &_diffSum,
                   double &_diffAvg);

    /// \brief Function that receives new image frames.
    /// \param[in] _image Image data.
    /// \param[in] _width Width of the image frame.
    /// \param[in] _height Height of the image frame.
    /// \param[in] _depth Depth of the image frame.
    /// \param[in] _format Pixel format.
    private: void OnNewFrame(const unsigned char *_image,
                             unsigned int _width, unsigned int _height,
                             unsigned int _depth,
                             const std::string &/*_format*/);

    /// \brief Get an image frame from a camera.
    /// \param[in] _cameraName Name of the camera to get a frame from.
    /// \param[out] _imgData Array that receives the image data.
    /// \param[in] _width Width of the image frame.
    /// \param[in] _height Height of the image frame.
    protected: void GetFrame(const std::string &_cameraName,
                   unsigned char **_imgData, unsigned int &_width,
                   unsigned int &_height);

    /// \brief Spawn a model from a msgs::Model and return ModelPtr.
    /// \param[in] _msg Model message.
    /// \return Pointer to model.
    protected: physics::ModelPtr SpawnModel(const msgs::Model &_msg);

    /// \brief Check that a pointer is not NULL. A function is created
    /// for this purpose, since ASSERT's cannot be called from non-void
    /// functions.
    /// \param[in] _ptr Pointer to verify is not NULL.
    protected: template<typename T>
      static void CheckPointer(boost::shared_ptr<T> _ptr)
      {
        ASSERT_TRUE(_ptr != NULL);
      }

    /// \brief Spawn a camera.
    /// \param[in] _modelName Name of the model.
    /// \param[in] _cameraName Name of the camera.
    /// \param[in] _pos Camera position.
    /// \param[in] _rpy Camera roll, pitch, yaw.
    /// \param[in] _width Output image width.
    /// \param[in] _height Output image height.
    /// \param[in] _rate Output Hz.
    /// \param[in] _noiseType Type of noise to apply.
    /// \param[in] _noiseMean Mean noise value.
    /// \param[in] _noiseStdDev Standard deviation of the noise.
    /// \param[in] _distortionK1 Distortion coefficient k1.
    /// \param[in] _distortionK2 Distortion coefficient k2.
    /// \param[in] _distortionK3 Distortion coefficient k3.
    /// \param[in] _distortionP1 Distortion coefficient P1.
    /// \param[in] _distortionP2 Distortion coefficient p2.
    /// \param[in] _cx Normalized optical center x, used for distortion.
    /// \param[in] _cy Normalized optical center y, used for distortion.
    protected: void SpawnCamera(const std::string &_modelName,
                   const std::string &_cameraName,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   unsigned int _width = 320, unsigned int _height = 240,
                   double _rate = 25,
                   const std::string &_noiseType = "",
                   double _noiseMean = 0.0, double _noiseStdDev = 0.0,
                   bool _distortion = false, double _distortionK1 = 0.0,
                   double _distortionK2 = 0.0, double _distortionK3 = 0.0,
                   double _distortionP1 = 0.0, double _distortionP2 = 0.0,
                   double _cx = 0.5, double _cy = 0.5);

    /// \brief Spawn a laser.
    /// \param[in] _modelName Name of the model.
    /// \param[in] _raySensorName Name of the laser.
    /// \param[in] _pos Camera position.
    /// \param[in] _rpy Camera roll, pitch, yaw.
    /// \param[in] _hMinAngle Horizontal min angle
    /// \param[in] _hMaxAngle Horizontal max angle
    /// \param[in] _minRange Min range
    /// \param[in] _maxRange Max range
    /// \param[in] _rangeResolution Resolution of the scan
    /// \param[in] _samples Number of samples.
    /// \param[in] _rate Output Hz.
    /// \param[in] _noiseType Type of noise to apply.
    /// \param[in] _noiseMean Mean noise value.
    /// \param[in] _noiseStdDev Standard deviation of the noise.
    protected: void SpawnRaySensor(const std::string &_modelName,
                   const std::string &_raySensorName,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   double _hMinAngle = -2.0, double _hMaxAngle = 2.0,
                   double _vMinAngle = -1.0, double _vMaxAngle = 1.0,
                   double _minRange = 0.08, double _maxRange = 10,
                   double _rangeResolution = 0.01, unsigned int _samples = 640,
                   unsigned int _vSamples = 1, double _hResolution = 1.0,
                   double _vResolution = 1.0,
                   const std::string &_noiseType = "", double _noiseMean = 0.0,
                   double _noiseStdDev = 0.0);

    /// \brief Spawn a gpu laser.
    /// \param[in] _modelName Name of the model.
    /// \param[in] _raySensorName Name of the laser.
    /// \param[in] _pos Camera position.
    /// \param[in] _rpy Camera roll, pitch, yaw.
    /// \param[in] _hMinAngle Horizontal min angle
    /// \param[in] _hMaxAngle Horizontal max angle
    /// \param[in] _minRange Min range
    /// \param[in] _maxRange Max range
    /// \param[in] _rangeResolution Resolution of the scan
    /// \param[in] _samples Number of samples.
    /// \param[in] _rate Output Hz.
    /// \param[in] _noiseType Type of noise to apply.
    /// \param[in] _noiseMean Mean noise value.
    /// \param[in] _noiseStdDev Standard deviation of the noise.
    protected: void SpawnGpuRaySensor(const std::string &_modelName,
                   const std::string &_raySensorName,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   double _hMinAngle = -2.0, double _hMaxAngle = 2.0,
                   double _minRange = 0.08, double _maxRange = 10,
                   double _rangeResolution = 0.01, unsigned int _samples = 640,
                   const std::string &_noiseType = "", double _noiseMean = 0.0,
                   double _noiseStdDev = 0.0);

    /// \brief Spawn an imu sensor laser.
    /// \param[in] _modelName Name of the model.
    /// \param[in] _imuSensorName Name of the imu sensor.
    /// \param[in] _pos Camera position.
    /// \param[in] _rpy Camera roll, pitch, yaw.
    /// \param[in] _noiseType Type of noise to apply.
    /// \param[in] _noiseMean Mean noise value.
    /// \param[in] _noiseStdDev Standard deviation of the noise.
    /// \param[in] _accelNoiseMean Acceleration based noise mean.
    /// \param[in] _accelNoiseStdDev Acceleration based noise standard
    /// deviation.
    /// \param[in] _accelBiasMean Acceleration mean bias
    /// \param[in] _accelBiasStdDev Acceleration standard deviation bias
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

    /// \brief Spawn an altimeter sensor on a link
    /// \param[in] _name Model name
    /// \param[in] _sensorName Sensor name
    /// \param[in] _collisionType Type of collision, box or cylinder
    /// \param[in] _topic Topic to publish altimeter data to
    /// \param[in] _pos World position
    /// \param[in] _rpy World rotation in Euler angles
    /// \param[in] _static True to make the model static
    protected: void SpawnUnitAltimeterSensor(const std::string &_name,
                   const std::string &_sensorName,
                   const std::string &_collisionType,
                   const std::string &_topic,
                   const ignition::math::Vector3d &_pos,
                   const ignition::math::Vector3d &_rpy,
                   bool _static = false);

    /// \brief Spawn a magnetometer sensor on a link
    /// \param[in] _name Model name
    /// \param[in] _sensorName Sensor name
    /// \param[in] _collisionType Type of collision, box or cylinder
    /// \param[in] _topic Topic to publish magnetometer data to
    /// \param[in] _pos World position
    /// \param[in] _rpy World rotation in Euler angles
    /// \param[in] _static True to make the model static
    protected: void SpawnUnitMagnetometerSensor(const std::string &_name,
                   const std::string &_sensorName,
                   const std::string &_collisionType,
                   const std::string &_topic,
                   const ignition::math::Vector3d &_pos,
                   const ignition::math::Vector3d &_rpy,
                   bool _static = false);

    /// \brief generate a gtest failure from a timeout error and display a
    /// log message about the problem.
    /// \param[in] log_msg: error msg related to the timeout
    /// \param[in] timeoutCS: failing period (in centiseconds)
    private: void launchTimeoutFailure(const char *_logMsg,
                                       const int _timeoutCS);

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
    protected: void WaitUntilEntitySpawn(const std::string &_name,
                                       unsigned int _sleepEach,
                                       int _retries);

    /// \brief Wait for a number of ms. and attempts until the sensor is spawned
    /// \param[in] _name Sensor name
    /// \param[in] _sleepEach Number of milliseconds to sleep in each iteration
    /// \param[in] _retries Number of iterations until give up
    protected: void WaitUntilSensorSpawn(const std::string &_name,
                                       unsigned int _sleepEach,
                                       int _retries);

    /// \brief Spawn a light.
    /// \param[in] _name Name for the light.
    /// \param[in] _size Type of light - "spot", "directional", or "point".
    /// \param[in] _pos Position for the light.
    /// \param[in] _rpy Roll, pitch, yaw for the light.
    /// \param[in] _diffuse Diffuse color of the light.
    /// \param[in] _specular Specular color of the light.
    /// \param[in] _direction Direction of the light ("spot" and "directional").
    /// \param[in] _attenuationRange Range of attenuation.
    /// \param[in] _attenuationConstant Constant component of attenuation
    /// \param[in] _attenuationLinear Linear component of attenuation
    /// \param[in] _attenuationQuadratic Quadratic component of attenuation
    /// \param[in] _spotInnerAngle Inner angle ("spot" only).
    /// \param[in] _spotOuterAngle Outer angle ("spot" only).
    /// \param[in] _spotFallOff Fall off ("spot" only).
    /// \param[in] _castShadows True to cast shadows.
    protected: void SpawnLight(const std::string &_name,
                   const std::string &_type,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   const common::Color &_diffuse = common::Color::White,
                   const common::Color &_specular = common::Color::White,
                   const math::Vector3 &_direction = -math::Vector3::UnitZ,
                   double _attenuationRange = 20,
                   double _attenuationConstant = 0.5,
                   double _attenuationLinear = 0.01,
                   double _attenuationQuadratic = 0.001,
                   double _spotInnerAngle = 0,
                   double _spotOuterAngle = 0,
                   double _spotFallOff = 0,
                   bool _castShadows = true);

    /// \brief Spawn a cylinder
    /// \param[in] _name Name for the model.
    /// \param[in] _pos Position for the model.
    /// \param[in] _rpy Roll, pitch, yaw for the model.
    /// \param[in] _static True to make the model static.
    protected: void SpawnCylinder(const std::string &_name,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   bool _static = false);

    /// \brief Spawn a sphere
    /// \param[in] _name Name for the model.
    /// \param[in] _pos Position for the model.
    /// \param[in] _rpy Roll, pitch, yaw for the model.
    /// \param[in] _static True to make the model static.
    /// \param[in] _wait True to wait for the sphere to spawn before
    /// returning.
    protected: void SpawnSphere(const std::string &_name,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   bool _wait = true, bool _static = false);

    /// \brief Spawn a sphere
    /// \param[in] _name Name for the model.
    /// \param[in] _pos Position for the model.
    /// \param[in] _rpy Roll, pitch, yaw for the model.
    /// \param[in] _cog Center of gravity.
    /// \param[in] _radius Sphere radius.
    /// \param[in] _static True to make the model static.
    /// \param[in] _wait True to wait for the sphere to spawn before
    /// returning.
    protected: void SpawnSphere(const std::string &_name,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   const math::Vector3 &_cog, double _radius,
                   bool _wait = true, bool _static = false);

    /// \brief Spawn a box.
    /// \param[in] _name Name for the model.
    /// \param[in] _size Size of the box.
    /// \param[in] _pos Position for the model.
    /// \param[in] _rpy Roll, pitch, yaw for the model.
    /// \param[in] _static True to make the model static.
    protected: void SpawnBox(const std::string &_name,
                   const math::Vector3 &_size, const math::Vector3 &_pos,
                   const math::Vector3 &_rpy, bool _static = false);

    /// \brief Spawn a triangle mesh.
    /// \param[in] _name Name for the model.
    /// \param[in] _modelPath Path to the mesh file.
    /// \param[in] _scale Scaling factor.
    /// \param[in] _pos Position for the model.
    /// \param[in] _rpy Roll, pitch, yaw for the model.
    /// \param[in] _static True to make the model static.
    protected: void SpawnTrimesh(const std::string &_name,
                   const std::string &_modelPath, const math::Vector3 &_scale,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   bool _static = false);

    /// \brief Spawn an empty link.
    /// \param[in] _name Name for the model.
    /// \param[in] _pos Position for the model.
    /// \param[in] _rpy Roll, pitch, yaw for the model.
    /// \param[in] _static True to make the model static.
    protected: void SpawnEmptyLink(const std::string &_name,
                   const math::Vector3 &_pos, const math::Vector3 &_rpy,
                   bool _static = false);

    /// \brief Spawn a model from file.
    /// \param[in] _filename File to load a model from.
    protected: void SpawnModel(const std::string &_filename);

    /// \brief Send a factory message based on an SDF string.
    /// \param[in] _sdf SDF string to publish.
    protected: void SpawnSDF(const std::string &_sdf);

    /// \brief Load a plugin.
    /// \param[in] _filename Plugin filename to load.
    /// \param[in] _name Name to associate with with the plugin.
    protected: void LoadPlugin(const std::string &_filename,
                               const std::string &_name);

    /// \brief Get a pointer to a model.
    /// \param[in] _name Name of the model to get.
    /// \return Pointer to the model, or NULL if the model was not found.
    protected: physics::ModelPtr GetModel(const std::string &_name);

    /// \brief Remove a model by name.
    /// \param[in] _name Name of the model to remove.
    protected: void RemoveModel(const std::string &_name);

    /// \brief Remove a plugin.
    /// \param[in] _name Name of the plugin to remove.
    protected: void RemovePlugin(const std::string &_name);

    /// \brief Get the current memory information.
    /// \param[out] _resident Resident memory.
    /// \param[out] _share Shared memory.
    protected: void GetMemInfo(double &_resident, double &_share);

    /// \brief Get unique string with a specified prefix.
    /// \param[in] _prefix Prefix for unique string.
    /// \return String with prefix and unique number as suffix.
    protected: std::string GetUniqueString(const std::string &_prefix);

    /// \brief Helper to record data to gtest xml output.
    /// \param[in] _name Name of data.
    /// \param[in] _data Floating point number to store.
    protected: void Record(const std::string &_name, const double _data);

    /// \brief Helper to record signal statistics to gtest xml output.
    /// \param[in] _prefix Prefix string for data names.
    /// \param[in] _stats Signal statistics to store.
    protected: void Record(const std::string &_prefix,
                           const math::SignalStats &_stats);

    /// \brief Helper to record Vector3 signal statistics to gtest xml output.
    /// \param[in] _prefix Prefix string for data names.
    /// \param[in] _stats Vector3 signal statistics to store.
    protected: void Record(const std::string &_prefix,
                           const math::Vector3Stats &_stats);

    /// \brief Pointer the Gazebo server.
    protected: Server *server;

    /// \brief Pointer the thread the runs the server.
    protected: boost::thread *serverThread;

    /// \brief Pointer to a node for communication.
    protected: transport::NodePtr node;

    /// \brief Pose subscription.
    protected: transport::SubscriberPtr poseSub;

    /// \brief World statistics subscription.
    protected: transport::SubscriberPtr statsSub;

    /// \brief Factory publisher.
    protected: transport::PublisherPtr factoryPub;

    /// \brief Request publisher.
    protected: transport::PublisherPtr requestPub;

    /// \brief Map of received poses.
    protected: std::map<std::string, math::Pose> poses;

    /// \brief Mutex to protect data structures that store messages.
    protected: boost::mutex receiveMutex;

    /// \brief Image data
    private: unsigned char **imgData;

    /// \brief Increments when images are received.
    private: int gotImage;

    /// \brief Current simulation time, real time, and pause time.
    protected: common::Time simTime, realTime, pauseTime;

    /// \brief Current percent realtime.
    private: double percentRealTime;

    /// \brief True if paused.
    private: bool paused;

    /// \brief True if server is running.
    private: bool serverRunning;

    /// \brief Counter for unique name generation.
    private: int uniqueCounter;
  };
}       // namespace gazebo
#endif  // define _GAZEBO_SERVER_FIXTURE_HH_
