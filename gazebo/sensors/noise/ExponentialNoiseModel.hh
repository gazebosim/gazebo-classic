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
    /// \class ExponentialNoiseModel
    /// \brief Ornstein-Uhlenbeck noise class
    class GAZEBO_VISIBLE ExponentialNoiseModel : public Noise
    {
        /// \brief Constructor.
        public: ExponentialNoiseModel();

        /// \brief Destructor.
        public: virtual ~ExponentialNoiseModel();

        // Documentation inherited.
        public: virtual void Load(sdf::ElementPtr _sdf);

        // Documentation inherited.
        public: virtual void Fini();

        // Documentation inherited.
        public: double ApplyImpl(double _in);

        /// \brief Accessor for mean.
        /// \return Rate of the exponentially-distributed noise.
        public: double GetRate() const;

        /// \brief Stochastic process standard deviation
        protected: double rate;

        /// \brief The current error value
        protected: double error;
    };
  }
}

#endif
