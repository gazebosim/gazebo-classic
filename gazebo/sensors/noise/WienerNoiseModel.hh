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

#ifndef _GAZEBO_WIENER_NOISE_MODEL_HH_
#define _GAZEBO_WIENER_NOISE_MODEL_HH_

#include <vector>
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/sensors/noise/Noise.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \class WienerNoiseModel
    /// \brief Ornstein-Uhlenbeck noise class
    class GAZEBO_VISIBLE WienerNoiseModel : public Noise
    {
        /// \brief Constructor.
        public: WienerNoiseModel();

        /// \brief Destructor.
        public: virtual ~WienerNoiseModel();

        // Documentation inherited.
        public: virtual void Load(sdf::ElementPtr _sdf);

        // Documentation inherited.
        public: virtual void Fini();

        // Documentation inherited.
        public: double ApplyImpl(double _in);

        /// \brief Accessor for mean.
        /// \return Mean of Ornstein-Uhlenbeck noise.
        public: double GetMean() const;

        /// \brief Accessor for stddev.
        /// \return Standard deviation of  Ornstein-Uhlenbeck noise.
        public: double GetStdDev() const;

        /// \brief Stochastic process mean
        protected: double mean;

        /// \brief Stochastic process standard deviation
        protected: double stdDev;

        /// \brief The current error value
        protected: double error;
    };
  }
}

#endif
