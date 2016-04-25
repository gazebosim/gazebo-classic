/*
--------------------------------------------------------------------------------
This source file is part of SkyX.
Visit http://www.paradise-studios.net/products/skyx/

Copyright (C) 2009-2012 Xavier Verguín González <xavyiy@gmail.com>

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.
--------------------------------------------------------------------------------
*/

#include <vector>
#include "VClouds/VClouds.h"

#include "SkyX.h"

namespace SkyX { namespace VClouds
{
VClouds::VClouds(Ogre::SceneManager *sm)
  : mSceneManager(sm)
    , mCamera(0)
    , mCreated(false)
    , mGeometrySettings(GeometrySettings())
    , mDistanceFallingParams(Ogre::Vector2(1, -1))
    , mRenderQueueGroups(RenderQueueGroups(
          Ogre::RENDER_QUEUE_MAIN, Ogre::RENDER_QUEUE_9))
    , mWindDirection(Ogre::Degree(0))
    , mWindSpeed(80.0f)
    , mWheater(Ogre::Vector2(0.5f, 1.0f))
    , mDelayedResponse(false)
    , mSunDirection(Ogre::Vector3(0, -1, 0))
    , mSunColor(Ogre::Vector3(1, 1, 1))
    , mAmbientColor(Ogre::Vector3(0.63f, 0.63f, 0.7f))
    , mLightResponse(Ogre::Vector4(0.25f, 0.2f, 1.0f, 0.1f))
    , mAmbientFactors(Ogre::Vector4(0.45f, 0.3f, 0.6f, 1))
    , mGlobalOpacity(1.0f)
    , mCloudFieldScale(1.0f)
    , mNoiseScale(4.2f)
    , mVisible(true)
    , mDataManager(new DataManager(this))
    , mGeometryManager(new GeometryManager(this))
    , mLightningManager(new LightningManager(this))
    , mCamerasData(std::vector<CameraData>())
{
}

VClouds::~VClouds()
{
  remove();
}

void VClouds::create()
{
  remove();

  mVolCloudsMaterial = static_cast<Ogre::MaterialPtr>(
      Ogre::MaterialManager::getSingleton().getByName("SkyX_VolClouds"));
  mVolCloudsLightningMaterial = static_cast<Ogre::MaterialPtr>(
      Ogre::MaterialManager::getSingleton().getByName(
        "SkyX_VolClouds_Lightning"));

  if (mVolCloudsMaterial.isNull() || mVolCloudsLightningMaterial.isNull())
  {
    SkyXLOG("Error while creating SkyX::VClouds::VClouds,"
            "materials are not found");
    return;
  }

  // Data manager
  mDataManager->create(128, 128, 20);

  // Geometry manager
  mGeometryManager->create(mGeometrySettings.Height,
      mGeometrySettings.Radius, mGeometrySettings.Alpha,
      mGeometrySettings.Beta, mGeometrySettings.NumberOfBlocks,
      mGeometrySettings.Na, mGeometrySettings.Nb, mGeometrySettings.Nc);

  mGeometryManager->getSceneNode()->setVisible(mVisible);

  mVolCloudsMaterial
    ->getTechnique(0)->getPass(0)->getVertexProgramParameters()->
    setNamedConstant("uRadius", mGeometrySettings.Radius);
  mVolCloudsLightningMaterial->
    getTechnique(0)->getPass(0)->getVertexProgramParameters()->
    setNamedConstant("uRadius", mGeometrySettings.Radius);

  // Lightning manager
  mLightningManager->create();

  mCreated = true;

  // Update material parameters
  setSunColor(mSunColor);
  setAmbientColor(mAmbientColor);
  setLightResponse(mLightResponse);
  setAmbientFactors(mAmbientFactors);

  // Set current wheater
  setWheater(mWheater.x, mWheater.y, mDelayedResponse);
}

void VClouds::create(const GeometrySettings& gs)
{
  // Update geometry settings
  mGeometrySettings = gs;

  create();
}

void VClouds::create(const Ogre::Vector2& Height, const float& Radius)
{
  // Update geometry params
  mGeometrySettings.Height = Height;
  mGeometrySettings.Radius = Radius;

  create();
}

void VClouds::remove()
{
  if (!mCreated)
  {
    return;
  }

  mDataManager->remove();
  mGeometryManager->remove();
  mLightningManager->remove();

  mCamera = 0;
  mCamerasData.clear();

  mVolCloudsMaterial.setNull();
  mVolCloudsLightningMaterial.setNull();

  mCreated = false;
}

void VClouds::update(const Ogre::Real& timeSinceLastFrame)
{
  if (!mCreated)
  {
    return;
  }

  mDataManager->update(timeSinceLastFrame);
  mGeometryManager->update(timeSinceLastFrame);
  mLightningManager->update(timeSinceLastFrame);

  if (mLightningManager->isEnabled())
  {
    mVolCloudsLightningMaterial->
      getTechnique(0)->getPass(0)->getFragmentProgramParameters()->
      setNamedConstant("uInterpolation", mDataManager->_getInterpolation());
    mVolCloudsLightningMaterial->
      getTechnique(0)->getPass(0)->getFragmentProgramParameters()->
      setNamedConstant("uSunDirection", -mSunDirection);
  }
  else
  {
    mVolCloudsMaterial->
      getTechnique(0)->getPass(0)->getFragmentProgramParameters()->
      setNamedConstant("uInterpolation", mDataManager->_getInterpolation());
    mVolCloudsMaterial->
      getTechnique(0)->getPass(0)->getFragmentProgramParameters()->
      setNamedConstant("uSunDirection", -mSunDirection);
  }
}

void VClouds::notifyCameraRender(Ogre::Camera* c,
    const Ogre::Real& timeSinceLastCameraFrame)
{
  if (!mCreated)
  {
    return;
  }

  mCamera = c;

  // Check if the camera is registered
  bool isRegistered = false;
  for (Ogre::uint32 k = 0; k < mCamerasData.size(); k++)
  {
    if (mCamerasData.at(k).camera == c)
    {
      isRegistered = true;
      break;
    }
  }

  if (!isRegistered)
  {
    mCamerasData.push_back(CameraData(c));
    SkyXLOG("VClouds warning: unregistered camera registered, "
            "manual unregistering is needed before camera destruction");
  }

  mGeometryManager->updateGeometry(c, timeSinceLastCameraFrame);
  mLightningManager->updateMaterial();
}

void VClouds::registerCamera(Ogre::Camera* c)
{
  for (Ogre::uint32 k = 0; k < mCamerasData.size(); k++)
  {
    if (mCamerasData.at(k).camera == c)
    {
      return;
    }
  }

  mCamerasData.push_back(CameraData(c));
}

void VClouds::unregisterCamera(Ogre::Camera* c)
{
  for (std::vector<CameraData>::iterator it = mCamerasData.begin();
       it != mCamerasData.end(); it++)
  {
    if ((*it).camera == c)
    {
      mCamerasData.erase(it);
      return;
    }
  }
}

void VClouds::setVisible(const bool& visible)
{
  mVisible = visible;

  if (!mCreated)
  {
    return;
  }

  mGeometryManager->getSceneNode()->setVisible(mVisible);
  mLightningManager->_setVisible(mVisible);
}

void VClouds::setEnabled(bool _enabled)
{
  if (!mCreated)
  {
    return;
  }

  bool visible = _enabled ? mVisible : false;
  mGeometryManager->getSceneNode()->setVisible(visible);
  mLightningManager->_setVisible(visible);
}

void VClouds::setRenderQueueGroups(const RenderQueueGroups& rqg)
{
  mRenderQueueGroups = rqg;

  if (!mCreated)
  {
    return;
  }

  mGeometryManager->_updateRenderQueueGroup(rqg.vclouds);
  mLightningManager->_updateRenderQueueGroup(rqg.vcloudsLightnings);
}

void VClouds::setSunColor(const Ogre::Vector3& SunColor)
{
  mSunColor = SunColor;

  if (!mCreated)
  {
    return;
  }

  mVolCloudsMaterial->getTechnique(0)->getPass(0)->
    getFragmentProgramParameters()
    ->setNamedConstant("uSunColor", mSunColor);
  mVolCloudsLightningMaterial->getTechnique(0)->getPass(0)->
    getFragmentProgramParameters()
    ->setNamedConstant("uSunColor", mSunColor);
}

void VClouds::setAmbientColor(const Ogre::Vector3& AmbientColor)
{
  mAmbientColor = AmbientColor;

  if (!mCreated)
  {
    return;
  }

  mVolCloudsMaterial->getTechnique(0)->getPass(0)->
    getFragmentProgramParameters()
    ->setNamedConstant("uAmbientColor", mAmbientColor);
  mVolCloudsLightningMaterial->getTechnique(0)->getPass(0)->
    getFragmentProgramParameters()
    ->setNamedConstant("uAmbientColor", mAmbientColor);
}

void VClouds::setLightResponse(const Ogre::Vector4& LightResponse)
{
  mLightResponse = LightResponse;

  if (!mCreated)
  {
    return;
  }

  mVolCloudsMaterial->getTechnique(0)->getPass(0)->
    getFragmentProgramParameters()
    ->setNamedConstant("uLightResponse", mLightResponse);
  mVolCloudsLightningMaterial->getTechnique(0)->getPass(0)->
    getFragmentProgramParameters()
    ->setNamedConstant("uLightResponse", mLightResponse);
}

void VClouds::setAmbientFactors(const Ogre::Vector4& AmbientFactors)
{
  mAmbientFactors = AmbientFactors;

  if (!mCreated)
  {
    return;
  }

  mVolCloudsMaterial->getTechnique(0)->getPass(0)->
    getFragmentProgramParameters()
    ->setNamedConstant("uAmbientFactors", mAmbientFactors);
  mVolCloudsLightningMaterial->getTechnique(0)->getPass(0)->
    getFragmentProgramParameters()
    ->setNamedConstant("uAmbientFactors", mAmbientFactors);
}

void VClouds::setWheater(const float& Humidity,
                         const float& AverageCloudsSize,
                         const bool& DelayedResponse)
{
  mWheater = Ogre::Vector2(Humidity, AverageCloudsSize);
  mDelayedResponse = DelayedResponse;

  if (!mCreated)
  {
    return;
  }

  mDataManager->setWheater(mWheater.x, mWheater.y, mDelayedResponse);
}
}}
