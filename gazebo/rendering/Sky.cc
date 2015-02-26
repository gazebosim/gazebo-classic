/*
 * Copyright 2011 Nate Koenig
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
#include "gazebo/rendering/skyx/include/SkyX.h"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Sky.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
Sky::Sky(ScenePtr _scene)
{
  this->scene = _scene;
  this->skyx = NULL;
  this->skyxController = NULL;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->skySub = this->node->Subscribe("~/sky", &Sky::OnSkyMsg, this);

  this->sdf.reset(new sdf::Element);
  sdf::initFile("sky.sdf", this->sdf);
}

/////////////////////////////////////////////////
Sky::~Sky()
{
}

/////////////////////////////////////////////////
void Sky::Load(sdf::ElementPtr _sdf)
{
  this->sdf->Copy(_sdf);
}

/////////////////////////////////////////////////
void Sky::Init()
{
  // Create SkyX
  if (!this->skyxController)
  {
    this->skyxController = new SkyX::BasicController();
    this->skyx = new SkyX::SkyX(this->scene->GetManager(),
                                this->skyxController);
    this->skyx->create();
    this->skyx->setTimeMultiplier(0);
  }

  /// Only enable the sky if the SDF contains a <sky> element.
  if (this->sdf->HasElement("sky"))
    this->SetSky();
}

/////////////////////////////////////////////////
void Sky::SetSky()
{
  // Set the time:
  // x = current time[0-24],
  // y = sunrise time[0-24],
  // z = sunset time[0-24]
  this->skyxController->setTime(Ogre::Vector3(
        this->sdf->GetValueDouble("time"),
        this->sdf->GetValueDouble("sunrise"),
        this->sdf->GetValueDouble("sunset")));

  // Moon phase in [-1,1] range, where -1 means fully covered Moon,
  // 0 clear Moon and 1 fully covered Moon
  this->skyxController->setMoonPhase(this->sdf->GetValueDouble("moon_phase"));

  this->skyx->getAtmosphereManager()->setOptions(
      SkyX::AtmosphereManager::Options(
        9.77501f,   // Inner radius
        10.2963f,   // Outer radius
        0.01f,      // Height position
        0.0017f,    // RayleighMultiplier
        0.000675f,  // MieMultiplier
        30,         // Sun Intensity
        Ogre::Vector3(0.57f, 0.54f, 0.44f),  // Wavelength
        -0.991f, 2.5f, 4));

  // Set the cloud parameters
  if (this->sdf->HasElement("clouds"))
  {
    sdf::ElementPtr cloudElem = this->sdf->GetElement("clouds");
    this->skyx->getVCloudsManager()->setWindSpeed(
        cloudElem->GetValueDouble("speed"));

    // Use true to update volumetric clouds based on the time multiplier
    this->skyx->getVCloudsManager()->setAutoupdate(false);

    SkyX::VClouds::VClouds *vclouds =
      this->skyx->getVCloudsManager()->getVClouds();

    // Set wind direction in radians
    vclouds->setWindDirection(Ogre::Radian(
          cloudElem->GetValueDouble("direction")));

    // Set the ambient color of the clouds
    vclouds->setAmbientColor(Ogre::Vector3(
          cloudElem->GetValueColor("ambient").r,
          cloudElem->GetValueColor("ambient").g,
          cloudElem->GetValueColor("ambient").b));

    // x = sun light power
    // y = sun beta multiplier
    // z = ambient color multiplier
    // w = distance attenuation
    vclouds->setLightResponse(
        Conversions::Convert(cloudElem->GetValueVector4("light_response")));

    vclouds->setAmbientFactors(
        Conversions::Convert(cloudElem->GetValueVector4("ambient_factors")));

    /// Set the density (humidity) and mean size of the clouds.
    vclouds->setWheater(
        math::clamp(cloudElem->GetValueDouble("humidity"), 0.0, 1.0),
        math::clamp(cloudElem->GetValueDouble("mean_size"), 0.0, 1.0), true);

    // Create VClouds
    if (!this->skyx->getVCloudsManager()->isCreated())
    {
      // SkyX::MeshManager::getSkydomeRadius(...) works for both finite and
      // infinite(=0) camera far clip distances
      this->skyx->getVCloudsManager()->create(2000.0);
      // this->skyx->getMeshManager()->getSkydomeRadius(mRenderingCamera));
    }
  }
  else
  {
    // Disable VClouds
    this->skyx->getVCloudsManager()->getVClouds()->setWheater(0, 0, false);
  }

  // Set the lightning parameters
  if (this->sdf->HasElement("lightning"))
  {
    sdf::ElementPtr lightningElem = this->sdf->GetElement("lightning");

    SkyX::VClouds::VClouds *vclouds =
      this->skyx->getVCloudsManager()->getVClouds();

    vclouds->getLightningManager()->setEnabled(true);

    // Set the time between lightning strikes
    vclouds->getLightningManager()->setAverageLightningApparitionTime(
        lightningElem->GetValueDouble("mean_time"));

    vclouds->getLightningManager()->setLightningColor(Ogre::Vector3(
        lightningElem->GetValueColor("color").r,
        lightningElem->GetValueColor("color").g,
        lightningElem->GetValueColor("color").b));

    vclouds->getLightningManager()->setLightningTimeMultiplier(
        lightningElem->GetValueDouble("time_multiplier"));
  }
  else
  {
    this->skyx->getVCloudsManager()->getVClouds()->
      getLightningManager()->setEnabled(false);
  }

  Ogre::Root::getSingletonPtr()->addFrameListener(this->skyx);

  this->skyx->update(0);
}

//////////////////////////////////////////////////
void Sky::ProcessSkyMsg(const msgs::Sky &_msg)
{
  if (_msg.has_time())
  {
    Ogre::Vector3 t = this->skyxController->getTime();
    t.x = math::clamp(_msg.time(), 0.0, 24.0);
    this->skyxController->setTime(t);
  }

  if (_msg.has_sunrise())
  {
    Ogre::Vector3 t = this->skyxController->getTime();
    t.y = math::clamp(_msg.sunrise(), 0.0, 24.0);
    this->skyxController->setTime(t);
  }

  if (_msg.has_sunset())
  {
    Ogre::Vector3 t = this->skyxController->getTime();
    t.z = math::clamp(_msg.sunset(), 0.0, 24.0);
    this->skyxController->setTime(t);
  }

  if (_msg.has_clouds())
  {
    if (!this->skyx->getVCloudsManager()->isCreated())
      this->skyx->getVCloudsManager()->create();

    SkyX::VClouds::VClouds *vclouds =
      this->skyx->getVCloudsManager()->getVClouds();

    if (_msg.clouds().enabled())
    {
      msgs::Sky::Clouds cloudsMsg = _msg.clouds();

      if (cloudsMsg.has_speed())
        vclouds->setWindSpeed(cloudsMsg.speed());

      if (cloudsMsg.has_direction())
        vclouds->setWindDirection(Ogre::Radian(cloudsMsg.direction()));

      if (cloudsMsg.has_ambient())
      {
        vclouds->setAmbientFactors(Ogre::Vector4(
              cloudsMsg.ambient().r(),
              cloudsMsg.ambient().g(),
              cloudsMsg.ambient().b(),
              cloudsMsg.ambient().a()));
      }

      if (cloudsMsg.has_humidity())
      {
        Ogre::Vector2 wheater = vclouds->getWheater();
        vclouds->setWheater(
            math::clamp(cloudsMsg.humidity(), 0.0, 1.0),
            wheater.y, false);
      }

      if (cloudsMsg.has_mean_size())
      {
        Ogre::Vector2 wheater = vclouds->getWheater();
        vclouds->setWheater(wheater.x,
            math::clamp(cloudsMsg.mean_size(), 0.0, 1.0), false);
      }
    }
    else
    {
      vclouds->setWheater(0, 0, false);
    }
  }

  if (_msg.has_lightning())
  {
    msgs::Sky::Lightning lightningMsg = _msg.lightning();
  }

  /// \todo: Add skyx to the root's frame listener: Ogre::Root::addFrameListener
  this->skyx->update(0.1);
}

/////////////////////////////////////////////////
void Sky::OnSkyMsg(ConstSkyPtr &_msg)
{
  this->ProcessSkyMsg(*_msg);
}

/////////////////////////////////////////////////
void Sky::AddCamera(CameraPtr _camera)
{
  if (this->skyx)
    _camera->GetRenderTarget()->addListener(this->skyx);
  else
  {
    gzerr << "Sky is improperly initialized. "
          << "Unable to add camera listner to the sky.\n";
  }
}
