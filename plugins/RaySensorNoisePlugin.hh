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

#ifndef _GAZEBO_RAY_SENSOR_NOISE_PLUGIN_HH_
#define _GAZEBO_RAY_SENSOR_NOISE_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief A Ray Sensor Noise Plugin
  class GAZEBO_VISIBLE RaySensorNoisePlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: RaySensorNoisePlugin();

    /// \brief Destructor
    public: virtual ~RaySensorNoisePlugin();

    /// \brief Callback for applying noise to a single sensor beam.
    /// \param[in] _in Input range value.
    /// \return Range value with noise added.
    public: virtual double OnApplyNoise(double _in);

    /// \brief Load the plugin
    /// \param[in] _parent Parent sensor
    /// \param[in] _sdf Parent sensor's sdf element.
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Apply fixed noise rate to each sensor beam.
    private: double fixedNoiseRate;

    /// \brief Sign of the noise value.
    private: int sign;
  };
}
#endif
