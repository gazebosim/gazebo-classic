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

    /// \brief Lens flare scale
    public: double scale = 1.0;
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

  if (_sdf->HasElement("scale"))
  {
    this->dataPtr->scale = _sdf->Get<double>("scale");
    if (this->dataPtr->scale < 0)
      gzerr << "Lens flare scale must be greater than 0" << std::endl;
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
void LensFlareSensorPlugin::AddLensFlare(rendering::CameraPtr _camera)
{
  if (!_camera)
    return;

  rendering::LensFlarePtr lensFlare;
  lensFlare.reset(new rendering::LensFlare);
  lensFlare->SetCamera(_camera);
  lensFlare->SetScale(this->dataPtr->scale);
  this->dataPtr->lensFlares.push_back(lensFlare);
}
