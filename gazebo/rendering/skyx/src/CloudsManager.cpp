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

#include "CloudsManager.h"

#include "SkyX.h"

namespace SkyX
{
  /// -------------- CloudLayer -----------------
  CloudLayer::CloudLayer(SkyX *s)
    : mSkyX(s),
    mOptions(Options()),
    mCloudLayerPass(0)
  {
    mAmbientGradient = ColorGradient();
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(1, 1, 1)*0.95f, 1.0f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.7, 0.7, 0.65), 0.625f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.6, 0.55, 0.4), 0.5625f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.6, 0.45, 0.3)*0.4, 0.5f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.5, 0.25, 0.25)*0.1, 0.45f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.2, 0.2, 0.3)*0.1, 0.35f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.2, 0.2, 0.5)*0.15, 0));

    mSunGradient = ColorGradient();
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(1, 1, 1)*0.95f, 1.0f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(1, 1, 1)*0.8, 0.75f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.8, 0.75, 0.55)*1.3, 0.5625f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.6, 0.5, 0.2)*0.75, 0.5f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.6, 0.5, 0.2)*0.35, 0.4725f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.5, 0.5, 0.5)*0.15, 0.45f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.2, 0.2, 0.25)*0.5, 0.3f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.5, 0.5, 0.5)*0.35, 0.0f));
  }

  CloudLayer::CloudLayer(SkyX *s, const Options& o)
    : mSkyX(s),
    mOptions(o),
    mCloudLayerPass(0)
  {
    mAmbientGradient = ColorGradient();
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(1, 1, 1)*0.95f, 1.0f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.7, 0.7, 0.65), 0.625f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.6, 0.55, 0.4), 0.5625f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.6, 0.45, 0.3)*0.4, 0.5f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.5, 0.25, 0.25)*0.1, 0.45f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.2, 0.2, 0.3)*0.1, 0.35f));
    mAmbientGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.2, 0.2, 0.5)*0.15, 0));

    mSunGradient = ColorGradient();
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(1, 1, 1)*0.95f, 1.0f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(1, 1, 1)*0.8, 0.75f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.8, 0.75, 0.55)*1.3, 0.5625f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.6, 0.5, 0.2)*0.75, 0.5f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.6, 0.5, 0.2)*0.35, 0.4725f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.5, 0.5, 0.5)*0.15, 0.45f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.2, 0.2, 0.25)*0.5, 0.3f));
    mSunGradient.addCFrame(
        ColorGradient::ColorFrame(Ogre::Vector3(0.5, 0.5, 0.5)*0.35, 0.0f));
  }

  CloudLayer::~CloudLayer()
  {
    _unregister();
  }

  void CloudLayer::_registerCloudLayer(Ogre::Pass* CloudLayerPass)
  {
    _unregister();

    CloudLayerPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    CloudLayerPass->setCullingMode(Ogre::CULL_NONE);
    CloudLayerPass->setLightingEnabled(false);
    CloudLayerPass->setDepthWriteEnabled(false);

    CloudLayerPass->setVertexProgram("SkyX_Clouds_VP");
    if (mSkyX->getLightingMode() == SkyX::LM_LDR)
    {
      CloudLayerPass->setFragmentProgram("SkyX_Clouds_LDR_FP");
    }
    else
    {
      CloudLayerPass->setFragmentProgram("SkyX_Clouds_HDR_FP");
    }

    // TODO
    CloudLayerPass->createTextureUnitState(
        "Clouds.png")->setTextureAddressingMode(
          Ogre::TextureUnitState::TAM_WRAP);
    CloudLayerPass->createTextureUnitState(
        "CloudsNormal.png")->setTextureAddressingMode(
          Ogre::TextureUnitState::TAM_WRAP);
    CloudLayerPass->createTextureUnitState(
        "CloudsTile.png")->setTextureAddressingMode(
          Ogre::TextureUnitState::TAM_WRAP);

    mCloudLayerPass = CloudLayerPass;

    _updatePassParameters();
    _updateInternalPassParameters();
  }

  void CloudLayer::_unregister()
  {
    if (mCloudLayerPass)
    {
      mCloudLayerPass->getParent()->removePass(mCloudLayerPass->getIndex());
      mCloudLayerPass = static_cast<Ogre::Pass*>(NULL);
    }
  }

  void CloudLayer::_updatePassParameters()
  {
    if (!mCloudLayerPass)
    {
      return;
    }

    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uScale", mOptions.Scale);
    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uHeight", mOptions.Height);

    float WindDirection_[2] = {mOptions.WindDirection.x,
      mOptions.WindDirection.y};

    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uWindDirection", WindDirection_, 1, 2);

    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uCloudLayerHeightVolume", mOptions.HeightVolume);
    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uCloudLayerVolumetricDisplacement",
          mOptions.VolumetricDisplacement);
    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uDetailAttenuation", mOptions.DetailAttenuation);
    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uDistanceAttenuation", mOptions.DistanceAttenuation);
  }

  void CloudLayer::_updateInternalPassParameters()
  {
    if (!mCloudLayerPass)
    {
      return;
    }

    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uExposure",
          mSkyX->getAtmosphereManager()->getOptions().Exposure);
    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uTime", mSkyX->_getTimeOffset()*
          mOptions.TimeMultiplier);
    /*
       mCloudLayerPass->getFragmentProgramParameters()
       ->setNamedConstant("uSunPosition",
       -mSkyX->getAtmosphereManager()->getSunDirection()*
       mSkyX->getMeshManager()->getSkydomeRadius());

       Ogre::Vector3 AmbientColor = Ogre::Vector3::ZERO;

       Ogre::Vector3 SunDir = -mSkyX->getAtmosphereManager()->getSunDirection();
       Ogre::Real Ang = 0;
       for (int k = 0; k < 3; k++)
       {
       Ogre::Vector2 Coords = Ogre::Vector2(Ogre::Math::Cos(Ang),
       Ogre::Math::Sin(Ang));
       Ang += 2*Ogre::Math::PI/3;
       AmbientColor += mSkyX->getAtmosphereManager()->getColorAt(
       Ogre::Vector3(Coords.x, mOptions.Height/mSkyX->getMeshManager()->
       getSkydomeRadius(), Coords.y).normalisedCopy());
       }

       AmbientColor /= 3;

       mCloudLayerPass->getFragmentProgramParameters()
       ->setNamedConstant("uAmbientLuminosity", AmbientColor*0.75f);

       float Mult = 1.5f;

       mCloudLayerPass->getFragmentProgramParameters()
       ->setNamedConstant("uSunColor", Ogre::Vector3(
       Ogre::Math::Clamp<Ogre::Real>(AmbientColor.x*Mult, 0, 1),
       Ogre::Math::Clamp<Ogre::Real>(AmbientColor.y*Mult, 0, 1),
       Ogre::Math::Clamp<Ogre::Real>(AmbientColor.z*Mult, 0, 1)));
       */

    // Ogre::Vector3 SunDir = mSkyX->getAtmosphereManager()->getSunDirection();
    // if (SunDir.y > 0.15f)
    // {
    //     SunDir = -SunDir;
    // }

    //    mCloudLayerPass->getFragmentProgramParameters()
    //        ->setNamedConstant("uSunPosition",
    //        -SunDir*mSkyX->getMeshManager()->getSkydomeRadius());

    float point = (mSkyX->getController()->getSunDirection().z + 1.0f) / 2.0f;

    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uSunColor", mSunGradient.getColor(point));
    mCloudLayerPass->getFragmentProgramParameters()
      ->setNamedConstant("uAmbientLuminosity",
          mAmbientGradient.getColor(point));
  }

  /// ------------- CloudsManager ---------------
  CloudsManager::CloudsManager(SkyX *s)
    : mSkyX(s)
  {
  }

  CloudsManager::~CloudsManager()
  {
    removeAll();
  }

  CloudLayer* CloudsManager::add(const CloudLayer::Options& o)
  {
    CloudLayer *NewCloudLayer = new CloudLayer(mSkyX, o);

    // TODO
    NewCloudLayer->_registerCloudLayer(static_cast<Ogre::MaterialPtr>(
          Ogre::MaterialManager::getSingleton().getByName(
            mSkyX->getGPUManager()->getSkydomeMaterialName()))
        ->getTechnique(0)->createPass());

    mCloudLayers.push_back(NewCloudLayer);

    bool changeOrder = false;

    // Short layers by height
    for (unsigned int k = 0; k < mCloudLayers.size(); k++)
    {
      if (k+1 < mCloudLayers.size())
      {
        if (mCloudLayers.at(k)->getOptions().Height <
            mCloudLayers.at(k+1)->getOptions().Height)
        {
          // Swap
          CloudLayer* cl = mCloudLayers.at(k);
          mCloudLayers.at(k) = mCloudLayers.at(k+1);
          mCloudLayers.at(k+1) = cl;

          changeOrder = true;
          k = 0;
        }
      }
    }

    if (changeOrder)
    {
      unregisterAll();
      registerAll();
    }

    return NewCloudLayer;
  }

  void CloudsManager::remove(CloudLayer* cl)
  {
    for (CloudLayersIt = mCloudLayers.begin();
        CloudLayersIt != mCloudLayers.end(); CloudLayersIt++)
    {
      if ((*CloudLayersIt) == cl)
      {
        delete (*CloudLayersIt);
        mCloudLayers.erase(CloudLayersIt);
        return;
      }
    }
  }

  void CloudsManager::removeAll()
  {
    for (CloudLayersIt = mCloudLayers.begin();
        CloudLayersIt != mCloudLayers.end(); CloudLayersIt++)
    {
      delete (*CloudLayersIt);
    }

    mCloudLayers.clear();
  }

  void CloudsManager::registerAll()
  {
    for (unsigned int k = 0; k < mCloudLayers.size(); k++)
    {
      mCloudLayers.at(k)->_registerCloudLayer(static_cast<Ogre::MaterialPtr>(
            Ogre::MaterialManager::getSingleton().getByName(
              mSkyX->getGPUManager()->getSkydomeMaterialName()))
          ->getTechnique(0)->createPass());
    }
  }

  void CloudsManager::unregister(CloudLayer* cl)
  {
    for (CloudLayersIt = mCloudLayers.begin();
        CloudLayersIt != mCloudLayers.end(); CloudLayersIt++)
    {
      if ((*CloudLayersIt) == cl)
      {
        (*CloudLayersIt)->_unregister();
      }
    }
  }

  void CloudsManager::unregisterAll()
  {
    for (CloudLayersIt = mCloudLayers.begin();
        CloudLayersIt != mCloudLayers.end(); CloudLayersIt++)
    {
      (*CloudLayersIt)->_unregister();
    }
  }

  void CloudsManager::update()
  {
    for (CloudLayersIt = mCloudLayers.begin();
        CloudLayersIt != mCloudLayers.end(); CloudLayersIt++)
    {
      (*CloudLayersIt)->_updateInternalPassParameters();
    }
  }
}
