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

#ifndef _GAZEBO_NOISE_HH_
#define _GAZEBO_NOISE_HH_

#include <vector>
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/sensors/SensorTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    class NoiseManager
    {
      public: static NoisePtr LoadNoiseModel(sdf::ElementPtr _sdf,
          const std::string &_sensorType = "");
    };

    /// \class Noise Noise.hh sensors/sensors.hh
    /// \brief Noise models for sensor output signals.
    class Noise
    {
      /// \brief Which noise types we support
      public: enum NoiseType
      {
        NONE,
        GAUSSIAN,
        GAUSSIAN_QUANTIZED,
        CUSTOM
      };

      /// \brief Constructor.
      public: Noise();

      /// \brief Destructor.
      public: ~Noise();

      /// \brief Load noise parameters from sdf.
      /// \param[in] _sdf SDF parameters.
      /// \param[in] _sensor Type of sensor.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Apply noise to input data value.
      /// \param[in] _in Input data value.
      /// \return Data with noise applied.
      public: double Apply(double _in) const;

      /// \brief Apply noise to input data value. This gets overriden by
      /// derived classes, and called by Apply.
      /// \param[in] _in Input data value.
      /// \return Data with noise applied.
      public: virtual double ApplyImpl(double _in) const;

      /// \brief Finalize the noise model
      public: virtual void Fini();

      /// \brief Accessor for NoiseType.
      /// \return Type of noise currently in use.
      public: NoiseType GetNoiseType() const;

      /// \brief Accessor for NoiseModel.
      /// \return Noise model currently in use.
      //public: NoiseModel *GetNoiseModel() const;

      /// \brief Set noise callback
      /// \param[in] Register a callback for applying a custom noise model.
      /// This is useful if users want to use their own noise model from a
      /// sensor plugin. Remember to set noise type to NoiseModel::CUSTOM.
      public: virtual void SetCustomNoiseCallback(
          boost::function<double(double)> _cb);

      /// \brief sdf data.
      private: sdf::ElementPtr sdf;

      /// \brief Which type of noise we're applying
      private: NoiseType type;

      /// \brief Noise model that will be applied to input data.
      //private: NoiseModel *noiseModel;

      /// \brief Apply noise to sensor data.
      /// \param[in] _subscriber Event callback.
      private: boost::function<double(double)> customNoiseCallback;
    };
    /// \}
  }
}
#endif
