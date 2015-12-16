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
#ifndef _GAZEBO_SENSORS_GAUSSIAN_NOISE_MODEL_HH_
#define _GAZEBO_SENSORS_GAUSSIAN_NOISE_MODEL_HH_

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
    // Forward declare protected data class
    class GaussianNoiseModelProtected;

    // Forward declare protected data class
    class ImageGaussianNoiseModelProtected;

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
      public: double ApplyImpl(const double _in);

      /// \brief Accessor for mean.
      /// \return Mean of Gaussian noise.
      /// \deprecated See Mean()
      public: double GetMean() const GAZEBO_DEPRECATED(7.0);

      /// \brief Accessor for mean.
      /// \return Mean of Gaussian noise.
      public: double Mean() const;

      /// \brief Accessor for stddev.
      /// \return Standard deviation of Gaussian noise.
      /// \deprecated See StdDev()
      public: double GetStdDev() const GAZEBO_DEPRECATED(7.0);

      /// \brief Accessor for stddev.
      /// \return Standard deviation of Gaussian noise.
      public: double StdDev() const;

      /// \brief Accessor for bias.
      /// \return Bias on output.
      /// \deprecated See Bias()
      public: double GetBias() const GAZEBO_DEPRECATED(7.0);

      /// \brief Accessor for bias.
      /// \return Bias on output.
      public: double Bias() const;

      /// Documentation inherited
      public: virtual void Print(std::ostream &_out) const;

      /// \brief Constructor for dervied classes.
      protected: GaussianNoiseModel(const GaussianNodeModelProtected &_dPtr);

      /// \internal
      /// \brief Protected data pointer
      protected: GaussianNoiseModelProtected *gaussDPtr;
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

      /// Documentation inherited
      public: virtual void Print(std::ostream &_out) const;

      /// \brief Get a pointer to the ogre compositor instance.
      /// \return Pointer to the ogre compositor instance.
      public: Ogre::CompositorInstance *CompositorInstance() const;

      /// \brief Get a pointer to the gaussian noise compositor listener.
      /// \return A pointer to the gaussian noise compositor listener.
      public: std::shared_ptr<GaussianNoiseCompositorListener>
              CompositorListener() const

      /// \internal
      /// \brief Protected data pointer.
      protected: ImageGaussianNoiseModelProtected *imgGaussDPtr;
    };
    /// \}
  }
}

#endif
