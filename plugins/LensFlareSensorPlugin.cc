/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <gazebo/rendering/LensFlare.hh>
#include <gazebo/rendering/RenderingIface.hh>

#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>

#include "LensFlareSensorPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class LensFlareSensorPlugin LensFlareSensorPlugin.hh
  /// \brief Private data for the LensFlareSensorPlugin class.
  class LensFlareSensorPluginPrivate
  {
    /// \brief Lens flare
    public: std::vector<rendering::LensFlarePtr> lensFlares;

    /// \brief Name of light that generates lens flare
    public: std::string lightName;

    /// \brief Lens flare scale
    public: double scale = 1.0;

    /// \brief Lens flare color
    public: ignition::math::Vector3d color
        = ignition::math::Vector3d(1.4, 1.2, 1.0);

    /// \brief Lens flare occlusion steps
    public: double occlusionSteps = 10.0;

    /// \brief Lens flare compositor name
    public: std::string compositorName;
  };
}

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(LensFlareSensorPlugin)

/////////////////////////////////////////////////
LensFlareSensorPlugin::LensFlareSensorPlugin()
    : dataPtr(new LensFlareSensorPluginPrivate)
{
}

/////////////////////////////////////////////////
LensFlareSensorPlugin::~LensFlareSensorPlugin()
{
}

/////////////////////////////////////////////////
void LensFlareSensorPlugin::Load(sensors::SensorPtr _sensor,
    sdf::ElementPtr _sdf)
{
  if (!_sensor)
  {
    gzerr << "Invalid sensor pointer." << std::endl;
    return;
  }
  if (!_sdf)
  {
    gzerr << "Invalid SDF pointer." << std::endl;
    return;
  }

  if (_sdf->HasElement("light_name"))
  {
    this->dataPtr->lightName = _sdf->Get<std::string>("light_name");
  }

  if (_sdf->HasElement("scale"))
  {
    this->dataPtr->scale = _sdf->Get<double>("scale");
    if (this->dataPtr->scale < 0)
      gzerr << "Lens flare scale must be greater than 0" << std::endl;
  }

  if (_sdf->HasElement("color"))
  {
    this->dataPtr->color = _sdf->Get<ignition::math::Vector3d>("color");
  }

  if (_sdf->HasElement("occlusion_steps"))
  {
    this->dataPtr->occlusionSteps = _sdf->Get<double>("occlusion_steps");
  }

  const std::string compositorName = "compositor";
  if (_sdf->HasElement(compositorName))
  {
    this->dataPtr->compositorName = _sdf->Get<std::string>(compositorName);
  }

  sensors::CameraSensorPtr cameraSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (cameraSensor)
  {
    this->AddLensFlare(cameraSensor->Camera());
    return;
  }
  else
  {
    sensors::MultiCameraSensorPtr multiCameraSensor =
      std::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);
    if (multiCameraSensor)
    {
      for (unsigned int i = 0; i < multiCameraSensor->CameraCount(); ++i)
      {
        this->AddLensFlare(multiCameraSensor->Camera(i));
      }
      return;
    }
  }

  gzerr << "LensFlarePlugin requires a [Multi]CameraSensor." << std::endl;
  return;
}

/////////////////////////////////////////////////
void LensFlareSensorPlugin::SetLightName(std::string _name)
{
  this->dataPtr->lightName = _name;

  for (auto flare : this->dataPtr->lensFlares)
  {
    flare->SetLightName(_name);
  }
}

/////////////////////////////////////////////////
void LensFlareSensorPlugin::SetScale(const double _scale)
{
  this->dataPtr->scale = _scale;

  for (auto flare : this->dataPtr->lensFlares)
  {
    flare->SetScale(_scale);
  }
}

/////////////////////////////////////////////////
void LensFlareSensorPlugin::SetColor(const ignition::math::Vector3d &_color)
{
  this->dataPtr->color = _color;

  for (auto flare : this->dataPtr->lensFlares)
  {
    flare->SetColor(_color);
  }
}

/////////////////////////////////////////////////
void LensFlareSensorPlugin::SetOcclusionSteps(double _occlusionSteps)
{
  this->dataPtr->occlusionSteps = _occlusionSteps;

  for (auto flare : this->dataPtr->lensFlares)
  {
    flare->SetOcclusionSteps(_occlusionSteps);
  }
}

/////////////////////////////////////////////////
void LensFlareSensorPlugin::AddLensFlare(rendering::CameraPtr _camera)
{
  if (!_camera)
    return;

  rendering::LensFlarePtr lensFlare;
  lensFlare.reset(new rendering::LensFlare);
  if (!this->dataPtr->compositorName.empty())
  {
    lensFlare->SetCompositorName(this->dataPtr->compositorName);
  }
  lensFlare->SetLightName(this->dataPtr->lightName);
  lensFlare->SetScale(this->dataPtr->scale);
  lensFlare->SetColor(this->dataPtr->color);
  lensFlare->SetOcclusionSteps(this->dataPtr->occlusionSteps);
  // SetCamera must be called last
  lensFlare->SetCamera(_camera);
  this->dataPtr->lensFlares.push_back(lensFlare);
}
