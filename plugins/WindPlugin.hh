/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_WINDPLUGIN_HH_
#define GAZEBO_PLUGINS_WINDPLUGIN_HH_

#include <ignition/math/Vector3.hh>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/Noise.hh"

namespace gazebo
{
  /// \brief A plugin that simulates a simple wind model.
  // The wind is described as a uniform worldwide model. So it is independant
  // from model position for simple computations. Its components are computed
  // separately:
  // - Horizontal amplitude:
  //      Low pass filtering on user input (complementary gain)
  //      + small local fluctuations
  //      + noise on value (noise amplitude is a factor of wind magnitude)
  //
  // - Horizontal direction:
  //      Low pass filtering on user input (complementary gain)
  //      + small local fluctuations
  //      + noise on value
  //
  // - Vertical amplitude:
  //      Noise proportionnal to wind magnitude.
  class GAZEBO_VISIBLE WindPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: WindPlugin();

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Get the global wind velocity.
    /// \param[in] _wind Reference to the wind.
    /// \param[in] _wind Pointer to an entity at which location the wind
    /// velocity is to be calculated.
    /// \return Wind's velocity at entity's location.
    public: ignition::math::Vector3d LinearVel(
            const physics::Wind *_wind,
            const physics::Entity *_entity);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief Time for wind to rise
    private: double characteristicTimeForWindRise = 1;

    /// \brief Wind amplitude
    private: double magnitudeSinAmplitudePercent = 0;

    /// \brief Wind period
    private: double magnitudeSinPeriod = 1;

    /// \brief Time for wind to change direction.
    private: double characteristicTimeForWindOrientationChange = 1;

    /// \brief Orientation amplitude
    private: double orientationSinAmplitude = 0;

    /// \brief Orientation period
    private: double orientationSinPeriod = 1;

    /// \brief period over characteristicTimeForWindRise
    private: double kMag = 0;

    /// \brief period over characteristicTimeForWindOrientationChange
    private: double kDir = 0;

    /// \brief Mean of the magnitude
    private: double magnitudeMean = 0;

    /// \brief Mean of the direction
    private: double directionMean = 0;

    /// \brief Noise added to magnitude
    private: sensors::NoisePtr noiseMagnitude;

    /// \brief Noise added to direction
    private: sensors::NoisePtr noiseDirection;

    /// \brief Noise added to Z axis
    private: sensors::NoisePtr noiseVertical;
  };
}

#endif
