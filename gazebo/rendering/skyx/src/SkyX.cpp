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

#ifdef _MSC_VER
#pragma warning(disable:4355)
#endif

#include "SkyX.h"

namespace SkyX
{
  SkyX::SkyX(Ogre::SceneManager* sm, Controller* c)
    : Ogre::FrameListener()
      , Ogre::RenderTargetListener()
      , mStarfield(true)
      , mLightingMode(LM_LDR)
      , mSceneManager(sm)
      , mController(c)
      , mCamera(0)
      , mMeshManager(new MeshManager(this))
      , mAtmosphereManager(new AtmosphereManager(this))
      , mGPUManager(new GPUManager(this))
      , mMoonManager(new MoonManager(this))
      , mCloudsManager(new CloudsManager(this))
      , mRenderQueueGroups(RenderQueueGroups(Ogre::RENDER_QUEUE_SKIES_EARLY,
            Ogre::RENDER_QUEUE_3,
            Ogre::RENDER_QUEUE_4))
      , mCreated(false)
      , mLastCameraPosition(Ogre::Vector3(0, 0, 0))
      , mLastCameraFarClipDistance(-1)
      , mInfiniteCameraFarClipDistance(100000)
      , mVisible(true)
      , mTimeMultiplier(0.1f)
      , mTimeOffset(0.0f)
      , mMoonEnabled(true)
      , mCloudsEnabled(true)
      , mEnabled(true)
  {
    // Need to be instanced here, when SkyX::mSceneManager is valid
    mVCloudsManager = new VCloudsManager(this);
  }

  SkyX::~SkyX()
  {
    remove();

    delete mMeshManager;
    delete mAtmosphereManager;
    delete mGPUManager;
    delete mMoonManager;
    delete mCloudsManager;
    delete mVCloudsManager;

    if (mController->getDeleteBySkyX())
    {
      delete mController;
    }
  }

  void SkyX::create()
  {
    if (mCreated)
    {
      return;
    }

    mGPUManager->_notifySkydomeMaterialChanged();

    mMeshManager->create();
    mMeshManager->setMaterialName(mGPUManager->getSkydomeMaterialName());

    mAtmosphereManager->_update(mAtmosphereManager->getOptions(), true);

    if (mStarfield)
    {
      mGPUManager->setGpuProgramParameter(GPUManager::GPUP_FRAGMENT,
                                          "uTime", mTimeOffset*0.5f, false);
    }

    Ogre::Vector3 sunDir = mController->getSunDirection();

    // Z-up conversion
    Ogre::Real ydir = sunDir.y;
    sunDir.y = sunDir.z;
    sunDir.z = ydir;

    mGPUManager->setGpuProgramParameter(GPUManager::GPUP_VERTEX,
        "uLightDir", sunDir);
    mGPUManager->setGpuProgramParameter(GPUManager::GPUP_FRAGMENT,
        "uLightDir", sunDir);

    mMoonManager->create();

    setVisible(mVisible);

    mLastCameraPosition = Ogre::Vector3(0, 0, 0);
    mLastCameraFarClipDistance = -1;

    // FIXME: Disable mMoonManager otherwise gpu range values get clipped.
    // issue #678
    mMoonManager->getMoonSceneNode()->setVisible(false);

    mCreated = true;
  }

  void SkyX::remove()
  {
    if (!mCreated)
    {
      return;
    }

    mCloudsManager->removeAll();
    mMeshManager->remove();
    mMoonManager->remove();
    mVCloudsManager->remove();

    mCamera = 0;

    mCreated = false;
  }

  void SkyX::update(const Ogre::Real& timeSinceLastFrame)
  {
    if (!mCreated || !this->mEnabled)
    {
      return;
    }

    if (!equal(mTimeMultiplier, 0.0f))
    {
      float timemultiplied = timeSinceLastFrame * mTimeMultiplier;

      mTimeOffset += timemultiplied;

      mController->update(timemultiplied);

      if (mStarfield)
      {
        mGPUManager->setGpuProgramParameter(GPUManager::GPUP_FRAGMENT,
            "uTime", mTimeOffset*0.5f, false);
      }
    }

    Ogre::Vector3 sunDir = mController->getSunDirection();

    // Z-up conversion
    Ogre::Real ydir = sunDir.y;
    sunDir.y = sunDir.z;
    sunDir.z = ydir;

    mGPUManager->setGpuProgramParameter(
        GPUManager::GPUP_VERTEX, "uLightDir", sunDir);
    mGPUManager->setGpuProgramParameter(
        GPUManager::GPUP_FRAGMENT, "uLightDir", sunDir);

    if (this->mMoonEnabled)
      mMoonManager->updateMoonPhase(mController->getMoonPhase());
    if (this->mCloudsEnabled)
    {
      mCloudsManager->update();
      mVCloudsManager->update(timeSinceLastFrame);
    }
  }

  void SkyX::notifyCameraRender(Ogre::Camera* c)
  {
    if (!mCreated || !this->mEnabled || !c)
    {
      return;
    }

    mCamera = c;

    if (mLastCameraPosition != c->getDerivedPosition())
    {
      mMeshManager->getSceneNode()->setPosition(mCamera->getDerivedPosition());

      mLastCameraPosition = mCamera->getDerivedPosition();
    }

    if (!equal(mLastCameraFarClipDistance, c->getFarClipDistance()))
    {
      mMeshManager->updateGeometry(c);

      mLastCameraFarClipDistance = mCamera->getFarClipDistance();
    }

    if (this->mMoonEnabled)
      mMoonManager->updateGeometry(c);
    if (this->mCloudsEnabled)
    {
      mVCloudsManager->notifyCameraRender(c);
    }
  }

  void SkyX::setVisible(const bool& visible)
  {
    mVisible = visible;

    if (!mCreated)
    {
      return;
    }

    mMeshManager->getSceneNode()->setVisible(mVisible);

    // Disable moon manager as it clips gpu laser range values
    // mMoonManager->getMoonSceneNode()->setVisible(mVisible);

    if (mVCloudsManager->isCreated())
    {
      mVCloudsManager->getVClouds()->setVisible(mVisible);
    }
  }

  void SkyX::setRenderQueueGroups(const RenderQueueGroups& rqg)
  {
    mRenderQueueGroups = rqg;

    mVCloudsManager->getVClouds()->setRenderQueueGroups(
        VClouds::VClouds::RenderQueueGroups(
          mRenderQueueGroups.vclouds, mRenderQueueGroups.vcloudsLightnings));

    if (!mCreated)
    {
      return;
    }

    mMeshManager->getEntity()->setRenderQueueGroup(mRenderQueueGroups.skydome);
    mMoonManager->getMoonBillboard()->setRenderQueueGroup(
        mRenderQueueGroups.skydome+1);
  }

  void SkyX::setLightingMode(const LightingMode& lm)
  {
    mLightingMode = lm;

    if (!mCreated)
    {
      return;
    }

    mGPUManager->_notifySkydomeMaterialChanged();

    // Update skydome material
    mMeshManager->setMaterialName(mGPUManager->getSkydomeMaterialName());
    // Update layered clouds material
    mCloudsManager->registerAll();
    // Update ground passes materials and update textures gamma correction
    mGPUManager->_updateFP();

    // Update parameters
    mAtmosphereManager->_update(mAtmosphereManager->getOptions(), true);
  }

  void SkyX::setStarfieldEnabled(const bool& Enabled)
  {
    mStarfield = Enabled;

    if (!mCreated)
    {
      return;
    }

    mGPUManager->_notifySkydomeMaterialChanged();

    // Update skydome material
    mMeshManager->setMaterialName(mGPUManager->getSkydomeMaterialName());

    // Update parameters
    mAtmosphereManager->_update(mAtmosphereManager->getOptions(), true);

    if (mStarfield)
    {
      mGPUManager->setGpuProgramParameter(
          GPUManager::GPUP_FRAGMENT, "uTime", mTimeOffset*0.5f, false);
    }

    Ogre::Vector3 sunDir = mController->getSunDirection();

    // Z-up conversion
    Ogre::Real ydir = sunDir.y;
    sunDir.y = sunDir.z;
    sunDir.z = ydir;

    mGPUManager->setGpuProgramParameter(
        GPUManager::GPUP_VERTEX, "uLightDir", sunDir);
    mGPUManager->setGpuProgramParameter(
        GPUManager::GPUP_FRAGMENT, "uLightDir", sunDir);
  }

  bool SkyX::frameStarted(const Ogre::FrameEvent& e)
  {
    update(e.timeSinceLastFrame);
    return true;
  }

  void SkyX::preViewportUpdate(const Ogre::RenderTargetViewportEvent& evt)
  {
    notifyCameraRender(evt.source->getCamera());
  }
}
