/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SENSORS_GAUSSIAN_NOISE_MODEL_PRIVATE_HH_
#define _GAZEBO_SENSORS_GAUSSIAN_NOISE_MODEL_PRIVATE_HH_

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief Protected data for GaussianNoiseModel
    class GaussianNoiseModelProtected
    {
      public: GaussianNoiseModelProtected()
      : mean(0.0),
        stdDev(0.0),
        bias(0.0),
        precision(0.0),
        quantized(false)
      {
      }

      /// \brief If type starts with GAUSSIAN, the mean of the distribution
      /// from which we sample when adding noise.
      public: double mean;

      /// \brief If type starts with GAUSSIAN, the standard deviation of the
      /// distribution from which we sample when adding noise.
      public: double stdDev;

      /// \brief If type starts with GAUSSIAN, the bias we'll add.
      public: double bias;

      /// \brief If type==GAUSSIAN_QUANTIZED, the precision to which
      /// the output signal is rounded.
      public: double precision;

      /// \brief True if the type is GAUSSIAN_QUANTIZED
      public: bool quantized;
    };

    /// \internal
    /// \brief Protected data for ImageGaussianNoiseModel
    class ImageGaussianNoiseModelProtected : public GaussianNoiseModelProtected
    {
      /// \brief Gaussian noise compositor.
      public: Ogre::CompositorInstance *gaussianNoiseInstance;

      /// \brief Gaussian noise compositor listener
      public: std::shared_ptr<GaussianNoiseCompositorListener>
        gaussianNoiseCompositorListener;
    };
  }
}
#endif
