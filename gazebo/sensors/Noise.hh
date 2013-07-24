/*
 * Copyright 2013 Open Source Robotics Foundation
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

#ifndef _NOISE_HH_
#define _NOISE_HH_

#include <vector>
#include <string>

#include <sdf/sdf.hh>

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class Noise Noise.hh sensors/sensors.hh
    /// \brief Noise models for sensor output signals.
    class Noise
    {
      /// \brief Constructor.
      public: Noise();

      /// \brief Destructor.
      public: ~Noise();

      /// \brief Load parameters from sdf.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Apply noise to input data value.
      /// \param[in] _in Input data value.
      /// \return Data with noise applied.
      public: double Apply(double _in);

      /// \brief Which noise type we support
      private: enum NoiseModelType
      {
        NONE,
        GAUSSIAN,
        GAUSSIAN_QUANTIZED
      };

      /// \brief 
      private: sdf::ElementPtr sdf;

      /// \brief Which type of noise we're applying
      private: enum NoiseModelType noiseType;

      /// \brief If noiseType starts with GAUSSIAN, the mean of the distibution
      /// from which we sample when adding noise.
      private: double mean;

      /// \brief If noiseType starts with GAUSSIAN, the standard devation of the
      /// distibution from which we sample when adding noise.
      private: double stdDev;

      /// \brief If noiseType starts with GAUSSIAN, the bias we'll add.
      private: double bias;

      /// \brief If noiseType==GAUSSIAN_QUANTIZED, the precision to which
      /// the output signal is rounded.
      private: double precision;
    };
    /// \}
  }
}
#endif
