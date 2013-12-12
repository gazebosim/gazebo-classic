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


#include <boost/math/special_functions/round.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Helpers.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/sensors/Noise.hh"

namespace gazebo
{

// We'll create an instance of this class for each camera, to be used to inject
// random values on each render call.
class GaussianNoiseCompositorListener
  : public Ogre::CompositorInstance::Listener
{
  /// \brief Constructor, setting mean and standard deviation.
  public: GaussianNoiseCompositorListener(double _mean, double _stddev):
      mean(_mean), stddev(_stddev) {}

  /// \brief Callback that OGRE will invoke for us on each render call
  public: virtual void notifyMaterialRender(unsigned int _pass_id,
                                            Ogre::MaterialPtr & _mat)
  {
    // modify material here (wont alter the base material!), called for
    // every drawn geometry instance (i.e. compositor render_quad)

    // Sample three values within the range [0,1.0] and set them for use in
    // the fragment shader, which will interpret them as offsets from (0,0)
    // to use when computing pseudo-random values.
    Ogre::Vector3 offsets(math::Rand::GetDblUniform(0.0, 1.0),
                          math::Rand::GetDblUniform(0.0, 1.0),
                          math::Rand::GetDblUniform(0.0, 1.0));
    // These calls are setting parameters that are declared in two places:
    // 1. media/materials/scripts/gazebo.material, in
    //    fragment_program Gazebo/GaussianCameraNoiseFS
    // 2. media/materials/scripts/camera_noise_gaussian_fs.glsl
    _mat->getTechnique(0)->getPass(_pass_id)->
      getFragmentProgramParameters()->
      setNamedConstant("offsets", offsets);
    _mat->getTechnique(0)->getPass(_pass_id)->
      getFragmentProgramParameters()->
      setNamedConstant("mean", (Ogre::Real)this->mean);
    _mat->getTechnique(0)->getPass(_pass_id)->
      getFragmentProgramParameters()->
      setNamedConstant("stddev", (Ogre::Real)this->stddev);
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
Noise::Noise()
  : type(NONE),
    noiseModel(NULL),
    customNoiseCallback(NULL)
{
}

//////////////////////////////////////////////////
Noise::~Noise()
{
  delete this->noiseModel;
}

//////////////////////////////////////////////////
void Noise::Load(sdf::ElementPtr _sdf, const std::string &_sensorType)
{
  this->sdf = _sdf;
  GZ_ASSERT(this->sdf != NULL, "this->sdf is NULL");
  std::string typeString = this->sdf->Get<std::string>("type");
  if (typeString == "none")
    this->type = NONE;
  else if (typeString == "gaussian")
    this->type = GAUSSIAN;
  else if (typeString == "gaussian_quantized")
    this->type = GAUSSIAN_QUANTIZED;
  else if (typeString == "custom")
    this->type = CUSTOM;
  else
  {
    gzerr << "Unrecognized noise type: [" << typeString << "]"
          << ", using default [none]" << std::endl;
    this->type = NONE;
  }

  if (this->type == GAUSSIAN ||
      this->type == GAUSSIAN_QUANTIZED)
  {
    if (_sensorType == "camera" || _sensorType == "depth" ||
      _sensorType == "multicamera")
    {
      this->noiseModel = new ImageGaussianNoiseModel();
    }
    else
      this->noiseModel = new GaussianNoiseModel();

    this->noiseModel->Load(this->sdf);
  }
}

//////////////////////////////////////////////////
double Noise::Apply(double _in) const
{
  if (this->type == NONE)
    return _in;
  else if (this->type == CUSTOM)
    return this->customNoiseCallback(_in);
  else
    return this->noiseModel->Apply(_in);
}

//////////////////////////////////////////////////
Noise::NoiseType Noise::GetNoiseType() const
{
  return this->type;
}

//////////////////////////////////////////////////
void Noise::SetCustomNoiseCallback(
    boost::function<double(double)> _cb)

{
  this->customNoiseCallback = _cb;
}

//////////////////////////////////////////////////
void Noise::Fini()
{
  this->noiseModel->Fini();
}

//////////////////////////////////////////////////
NoiseModel *Noise::GetNoiseModel() const
{
  return this->noiseModel;
}


//////////////////////////////////////////////////
NoiseModel::NoiseModel()
{
}

//////////////////////////////////////////////////
void NoiseModel::Load(sdf::ElementPtr /*_sdf*/)
{
}

//////////////////////////////////////////////////
void NoiseModel::Fini()
{
}

//////////////////////////////////////////////////
double NoiseModel::Apply(double _in) const
{
  return _in;
}

//////////////////////////////////////////////////
GaussianNoiseModel::GaussianNoiseModel()
  : NoiseModel(),
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
  this->mean = _sdf->Get<double>("mean");
  this->stdDev = _sdf->Get<double>("stddev");
  // Sample the bias
  double biasMean = 0;
  double biasStdDev = 0;
  if (_sdf->HasElement("bias_mean"))
    biasMean = _sdf->Get<double>("bias_mean");
  if (_sdf->HasElement("bias_stddev"))
    biasStdDev = _sdf->Get<double>("bias_stddev");
  this->bias = math::Rand::GetDblNormal(biasMean, biasStdDev);
  // With equal probability, we pick a negative bias (by convention,
  // rateBiasMean should be positive, though it would work fine if
  // negative).
  if (math::Rand::GetDblUniform() < 0.5)
    this->bias = -this->bias;
  gzlog << "applying Gaussian noise model with mean " << this->mean
    << ", stddev " << this->stdDev
    << ", bias " << this->bias << std::endl;

  if (_sdf->HasElement("precision"))
  {
    this->precision = _sdf->Get<double>("precision");
    this->quantized = true;
  }
}

//////////////////////////////////////////////////
void GaussianNoiseModel::Fini()
{
}

//////////////////////////////////////////////////
double GaussianNoiseModel::Apply(double _in) const
{
  double output = 0.0;

  double whiteNoise = math::Rand::GetDblNormal(this->mean, this->stdDev);
  output = _in + this->bias + whiteNoise;
  if (this->quantized)
  {
    // Apply this->precision
    if (!math::equal(this->precision, 0.0, 1e-6))
    {
      output = boost::math::round(output / this->precision) * this->precision;
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
void ImageGaussianNoiseModel::Init(rendering::Camera *_camera)
{
  this->camera = _camera;

  this->gaussianNoiseCompositorListener.reset(new
        GaussianNoiseCompositorListener(this->mean, this->stdDev));

  this->gaussianNoiseInstance =
    Ogre::CompositorManager::getSingleton().addCompositor(
      this->camera->GetViewport(), "CameraNoise/Gaussian");
  this->gaussianNoiseInstance->setEnabled(true);
  // gaussianNoiseCompositorListener was allocated in Load()
  this->gaussianNoiseInstance->addListener(
    this->gaussianNoiseCompositorListener.get());

}

//////////////////////////////////////////////////
void ImageGaussianNoiseModel::Fini()
{
  if (this->gaussianNoiseCompositorListener)
  {
    this->gaussianNoiseInstance->removeListener(
      this->gaussianNoiseCompositorListener.get());
  }
}
