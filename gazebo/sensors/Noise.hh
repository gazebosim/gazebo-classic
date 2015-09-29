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

#ifndef _GAZEBO_NOISE_HH_
#define _GAZEBO_NOISE_HH_

#include <vector>
#include <string>

#include <boost/function.hpp>
#include <sdf/sdf.hh>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class NoiseFactory Noise.hh sensors/sensors.hh
    /// \brief Use this noise manager for creating and loading noise models.
    class GAZEBO_VISIBLE NoiseFactory
    {
      /// \brief Load a noise model based on the input sdf parameters and
      /// sensor type.
      /// \param[in] _sdf Noise sdf parameters.
      /// \param[in] _sensorType Type of sensor. This is currently used to
      /// distinguish between image and non image sensors in order to create
      /// the appropriate noise model.
      /// \return Pointer to the noise model created.
      public: static NoisePtr NewNoiseModel(sdf::ElementPtr _sdf,
          const std::string &_sensorType = "");
    };

    /// \class Noise Noise.hh
    /// \brief Noise models for sensor output signals.
    class GAZEBO_VISIBLE Noise
    {
      /// \brief Which noise types we support
      public: enum NoiseType
      {
        NONE,
        CUSTOM,
        GAUSSIAN
      };

      /// \brief Constructor. This should not be called directly unless creating
      /// an empty noise model. Use NoiseFactory::NewNoiseModel to instantiate
      /// a new noise model.
      /// \param[in] _type Type of noise model.
      /// \sa NoiseFactory::NewNoiseModel
      public: explicit Noise(NoiseType _type);

      /// \brief Destructor.
      public: virtual ~Noise();

      /// \brief Load noise parameters from sdf.
      /// \param[in] _sdf SDF parameters.
      /// \param[in] _sensor Type of sensor.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Apply noise to input data value.
      /// \param[in] _in Input data value.
      /// \return Data with noise applied.
      public: double Apply(double _in);

      /// \brief Apply noise to input data value. This gets overriden by
      /// derived classes, and called by Apply.
      /// \param[in] _in Input data value.
      /// \return Data with noise applied.
      public: virtual double ApplyImpl(double _in);

      /// \brief Finalize the noise model
      public: virtual void Fini();

      /// \brief Accessor for NoiseType.
      /// \return Type of noise currently in use.
      public: NoiseType GetNoiseType() const;

      /// \brief Register a custom noise callback.
      /// \param[in] _cb Callback function for applying a custom noise model.
      /// This is useful if users want to use their own noise model from a
      /// sensor plugin.
      public: virtual void SetCustomNoiseCallback(
          boost::function<double (double)> _cb);

      /// \brief Set camera needed to create image noise. This is only needed
      /// for image sensors, i.e. camera/multicamera/depth sensors, which use
      /// shaders for more efficient noise generation.
      /// \param[in] _camera Camera associated to an image sensor
      public: virtual void SetCamera(rendering::CameraPtr _camera);

      /// \brief Which type of noise we're applying
      private: NoiseType type;

      /// \brief Noise sdf element.
      private: sdf::ElementPtr sdf;

      /// \brief Callback function for applying custom noise to sensor data.
      private: boost::function<double (double)> customNoiseCallback;
    };
    /// \}
  }
}
#endif
