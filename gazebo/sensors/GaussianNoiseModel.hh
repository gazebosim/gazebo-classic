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

#ifndef _GAZEBO_GAUSSIAN_NOISE_MODEL_HH_
#define _GAZEBO_GAUSSIAN_NOISE_MODEL_HH_

#include <vector>
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/sensors/Noise.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class CompositorInstance;
}

namespace gazebo
{
  class GaussianNoiseCompositorListener;

  namespace sensors
  {
    /// \class GaussianNoiseModel
    /// \brief Gaussian noise class
    class GAZEBO_VISIBLE GaussianNoiseModel : public Noise
    {
        /// \brief Constructor.
        public: GaussianNoiseModel();

        /// \brief Destructor.
        public: virtual ~GaussianNoiseModel();

        // Documentation inherited.
        public: virtual void Load(sdf::ElementPtr _sdf);

        // Documentation inherited.
        public: virtual void Fini();

        // Documentation inherited.
        public: double ApplyImpl(double _in);

        /// \brief Accessor for mean.
        /// \return Mean of Gaussian noise.
        public: double GetMean() const;

        /// \brief Accessor for stddev.
        /// \return Standard deviation of Gaussian noise.
        public: double GetStdDev() const;

        /// \brief Accessor for bias.
        /// \return Bias on output.
        public: double GetBias() const;

        /// \brief If type starts with GAUSSIAN, the mean of the distribution
        /// from which we sample when adding noise.
        protected: double mean;

        /// \brief If type starts with GAUSSIAN, the standard deviation of the
        /// distribution from which we sample when adding noise.
        protected: double stdDev;

        /// \brief If type starts with GAUSSIAN, the bias we'll add.
        protected: double bias;

        /// \brief If type==GAUSSIAN_QUANTIZED, the precision to which
        /// the output signal is rounded.
        protected: double precision;

        /// \brief True if the type is GAUSSIAN_QUANTIZED
        protected: bool quantized;
    };

    /// \class GaussianNoiseModel
    /// \brief Gaussian noise class for image sensors
    class GAZEBO_VISIBLE ImageGaussianNoiseModel : public GaussianNoiseModel
    {
      /// \brief Constructor.
      public: ImageGaussianNoiseModel();

      /// \brief Destructor.
      public: virtual ~ImageGaussianNoiseModel();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Fini();

      // Documentation inherited.
      public: virtual void SetCamera(rendering::CameraPtr _camera);

      /// \brief Gaussian noise compositor.
      public: Ogre::CompositorInstance *gaussianNoiseInstance;

      /// \brief Gaussian noise compositor listener
      public: boost::shared_ptr<GaussianNoiseCompositorListener>
        gaussianNoiseCompositorListener;
    };
    /// \}
  }
}

#endif
