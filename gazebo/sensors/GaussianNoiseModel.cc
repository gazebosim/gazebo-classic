/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <ignition/math/Helpers.hh>
#include <ignition/math/Rand.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/sensors/GaussianNoiseModel.hh"

namespace gazebo
{
  // We'll create an instance of this class for each camera, to be used to
  // inject random values on each render call.
  class GaussianNoiseCompositorListener
    : public Ogre::CompositorInstance::Listener
  {
    /// \brief Constructor, setting mean and standard deviation.
    public: GaussianNoiseCompositorListener(double _mean, double _stddev):
        mean(_mean), stddev(_stddev) {}

    /// \brief Callback that OGRE will invoke for us on each render call
    /// \param[in] _passID OGRE material pass ID.
    /// \param[in] _mat Pointer to OGRE material.
    public: virtual void notifyMaterialRender(unsigned int _passId,
                                              Ogre::MaterialPtr &_mat)
    {
      GZ_ASSERT(!_mat.isNull(), "Null OGRE material");
      // modify material here (wont alter the base material!), called for
      // every drawn geometry instance (i.e. compositor render_quad)

      // Sample three values within the range [0,1.0] and set them for use in
      // the fragment shader, which will interpret them as offsets from (0,0)
      // to use when computing pseudo-random values.
      Ogre::Vector3 offsets(ignition::math::Rand::DblUniform(0.0, 1.0),
                            ignition::math::Rand::DblUniform(0.0, 1.0),
                            ignition::math::Rand::DblUniform(0.0, 1.0));
      // These calls are setting parameters that are declared in two places:
      // 1. media/materials/scripts/gazebo.material, in
      //    fragment_program Gazebo/GaussianCameraNoiseFS
      // 2. media/materials/scripts/camera_noise_gaussian_fs.glsl
      Ogre::Technique *technique = _mat->getTechnique(0);
      GZ_ASSERT(technique, "Null OGRE material technique");
      Ogre::Pass *pass = technique->getPass(_passId);
      GZ_ASSERT(pass, "Null OGRE material pass");
      Ogre::GpuProgramParametersSharedPtr params =
          pass->getFragmentProgramParameters();
      GZ_ASSERT(!params.isNull(), "Null OGRE material GPU parameters");

      params->setNamedConstant("offsets", offsets);
      params->setNamedConstant("mean", static_cast<Ogre::Real>(this->mean));
      params->setNamedConstant("stddev", static_cast<Ogre::Real>(this->stddev));
    }

    /// \brief Mean that we'll pass down to the GLSL fragment shader.
    private: double mean;
    /// \brief Standard deviation that we'll pass down to the GLSL fragment
    /// shader.
    private: double stddev;
  };
}  // namespace gazebo

using namespace gazebo;
using namespace sensors;

//////////////////////////////////////////////////
GaussianNoiseModel::GaussianNoiseModel()
  : Noise(Noise::GAUSSIAN),
    mean(0.0),
    stdDev(0.0),
    bias(0.0),
    precision(0.0),
    quantized(false)
{
}

//////////////////////////////////////////////////
GaussianNoiseModel::~GaussianNoiseModel()
{
}

//////////////////////////////////////////////////
void GaussianNoiseModel::Load(sdf::ElementPtr _sdf)
{
  Noise::Load(_sdf);

  this->mean = _sdf->Get<double>("mean");
  this->stdDev = _sdf->Get<double>("stddev");
  // Sample the bias
  double biasMean = 0;
  double biasStdDev = 0;
  if (_sdf->HasElement("bias_mean"))
    biasMean = _sdf->Get<double>("bias_mean");
  if (_sdf->HasElement("bias_stddev"))
    biasStdDev = _sdf->Get<double>("bias_stddev");
  this->bias = ignition::math::Rand::DblNormal(biasMean, biasStdDev);
  // With equal probability, we pick a negative bias (by convention,
  // rateBiasMean should be positive, though it would work fine if
  // negative).
  if (ignition::math::Rand::DblUniform() < 0.5)
    this->bias = -this->bias;

  /// \todo Remove this, and use Noise::Print. See ImuSensor for an example
  gzlog << "applying Gaussian noise model with mean " << this->mean
    << ", stddev " << this->stdDev
    << ", bias " << this->bias << std::endl;

  if (_sdf->HasElement("precision"))
  {
    this->precision = _sdf->Get<double>("precision");
    if (this->precision < 0)
    {
      gzerr << "Noise precision cannot be less than 0" << std::endl;
    }
    else if (!ignition::math::equal(this->precision, 0.0, 1e-6))
    {
      this->quantized = true;
    }
  }
}

//////////////////////////////////////////////////
void GaussianNoiseModel::Fini()
{
  Noise::Fini();
}

//////////////////////////////////////////////////
double GaussianNoiseModel::ApplyImpl(double _in)
{
  // Add independent (uncorrelated) Gaussian noise to each input value.
  double whiteNoise = ignition::math::Rand::DblNormal(this->mean, this->stdDev);
  double output = _in + this->bias + whiteNoise;
  if (this->quantized)
  {
    // Apply this->precision
    if (!ignition::math::equal(this->precision, 0.0, 1e-6))
    {
      output = std::round(output / this->precision) * this->precision;
    }
  }
  return output;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::GetMean() const
{
  return this->mean;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::GetStdDev() const
{
  return this->stdDev;
}

//////////////////////////////////////////////////
double GaussianNoiseModel::GetBias() const
{
  return this->bias;
}

//////////////////////////////////////////////////
void GaussianNoiseModel::Print(std::ostream &_out) const
{
  _out << "Gaussian noise, mean[" << this->mean << "], "
    << "stdDev[" << this->stdDev << "] "
    << "bias[" << this->bias << "] "
    << "precision[" << this->precision << "] "
    << "quantized[" << this->quantized << "]";
}

//////////////////////////////////////////////////
ImageGaussianNoiseModel::ImageGaussianNoiseModel()
  : GaussianNoiseModel()
{
}

//////////////////////////////////////////////////
ImageGaussianNoiseModel::~ImageGaussianNoiseModel()
{
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::Load(sdf::ElementPtr _sdf)
{
  GaussianNoiseModel::Load(_sdf);
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::SetCamera(rendering::CameraPtr _camera)
{
  GZ_ASSERT(_camera, "Unable to apply gaussian noise, camera is null");

  this->gaussianNoiseCompositorListener.reset(new
        GaussianNoiseCompositorListener(this->mean, this->stdDev));

  this->gaussianNoiseInstance =
    Ogre::CompositorManager::getSingleton().addCompositor(
      _camera->OgreViewport(), "CameraNoise/Gaussian");
  this->gaussianNoiseInstance->setEnabled(true);
  this->gaussianNoiseInstance->addListener(
    this->gaussianNoiseCompositorListener.get());
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::Fini()
{
  GaussianNoiseModel::Fini();
}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::Print(std::ostream &_out) const
{
  _out << "Image Gaussian noise, mean[" << this->mean << "], "
    << "stdDev[" << this->stdDev << "] "
    << "bias[" << this->bias << "] "
    << "precision[" << this->precision << "] "
    << "quantized[" << this->quantized << "]";
}
