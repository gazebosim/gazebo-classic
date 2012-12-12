/*
--------------------------------------------------------------------------------
This source file is part of SkyX.
Visit http://www.paradise-studios.net/products/skyx/

Copyright (C) 2009-2012 Xavier Verguín González <xavyiy@gmail.com>

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License,  or (at your option) any later
version.

This program is distributed in the hope that it will be useful,  but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not,  write to the Free Software Foundation,  Inc.,  59 Temple
Place - Suite 330,  Boston,  MA 02111-1307,  USA,  or go to
http://www.gnu.org/copyleft/lesser.txt.
--------------------------------------------------------------------------------
*/

#include <vector>
#include "VClouds/LightningManager.h"

#include "VClouds/VClouds.h"

namespace SkyX { namespace VClouds
{
  LightningManager::LightningManager(VClouds* vc)
    : mVClouds(vc)
      ,  mLightnings(std::vector<Lightning*>())
      ,  mSceneNodes(std::vector<Ogre::SceneNode*>())
      ,  mEnabled(false)
      ,  mLightningColor(Ogre::Vector3(1,  0.925f,  0.85f))
      ,  mLightningTimeMultiplier(2.0f)
      ,  mAverageLightningApparitionTime(1.5f)
      ,  mRemainingTime(1.5f)
      ,  mVolCloudsLightningMaterial(Ogre::MaterialPtr())
      ,  mLightningMaterial(Ogre::MaterialPtr())
      ,  mListeners(std::vector<Listener*>())
      ,  mCreated(false)
  {
  }

  LightningManager::~LightningManager()
  {
    remove();
  }

  void LightningManager::create()
  {
    remove();

    mVolCloudsLightningMaterial = static_cast<Ogre::MaterialPtr>(
        Ogre::MaterialManager::getSingleton().getByName(
          "SkyX_VolClouds_Lightning"));
    mLightningMaterial = static_cast<Ogre::MaterialPtr>(
        Ogre::MaterialManager::getSingleton().getByName("SkyX_Lightning"));

    if (mLightningMaterial.isNull())
    {
      SkyXLOG("Error while creating SkyX::VClouds::LightningManager,  "
              "material not found");
      return;
    }

    if (mEnabled)
    {
      mVClouds->getGeometryManager()->_setMaterialName(
          "SkyX_VolClouds_Lightning");
    }
    else
    {
      mVClouds->getGeometryManager()->_setMaterialName("SkyX_VolClouds");
    }

    mCreated = true;

    setLightningColor(mLightningColor);
  }

  void LightningManager::remove()
  {
    if (!mCreated)
    {
      return;
    }

    for (Ogre::uint32 k = 0; k < mLightnings.size(); k++)
    {
      delete mLightnings.at(k);

      mVClouds->getSceneManager()->destroySceneNode(mSceneNodes.at(k));
    }

    mLightnings.clear();
    mSceneNodes.clear();

    removeListeners();

    mVolCloudsLightningMaterial.setNull();
    mLightningMaterial.setNull();

    mCreated = false;
  }

  void LightningManager::update(const Ogre::Real& timeSinceLastFrame)
  {
    if (!mCreated)
    {
      return;
    }

    if (mEnabled)
    {
      mRemainingTime -= timeSinceLastFrame;

      if (mRemainingTime <= 0)
      {
        mRemainingTime = Ogre::Math::RangeRandom(0,
            2*mAverageLightningApparitionTime);

        // Select a random camera to place the lightning
        if (!mVClouds->_getCamerasData().empty())
        {
          Ogre::Camera* c = mVClouds->_getCamerasData().at(
              mVClouds->_getCamerasData().size()*0.999).camera;

          Ogre::Real prob = Ogre::Math::RangeRandom(0,  1);

          // Cloud-to-ground
          if (prob < 0.5)
          {
            addLightning(
                // Ray position
                Ogre::Vector3(c->getDerivedPosition().x +
                  Ogre::Math::RangeRandom(-c->getFarClipDistance()*0.5,
                    c->getFarClipDistance()*0.5)/Ogre::Math::RangeRandom(1, 5),
                  c->getDerivedPosition().y + Ogre::Math::RangeRandom(
                    -c->getFarClipDistance()*0.5, c->getFarClipDistance()*0.5)/
                  Ogre::Math::RangeRandom(1, 5),
                  mVClouds->getGeometrySettings().Height.x +
                  0.2*mVClouds->getGeometrySettings().Height.y),
                // Ray direction
                Ogre::Vector3(0,  0, -1),
                // Ray length
                mVClouds->getGeometrySettings().Height.x +
                0.1*mVClouds->getGeometrySettings().Height.y);
          }
          // Cloud-to-cloud
          else if (prob < 0.7)
          {
            addLightning(
                // Ray position
                Ogre::Vector3(c->getDerivedPosition().x +
                  Ogre::Math::RangeRandom(-c->getFarClipDistance()*0.5,
                    c->getFarClipDistance()*0.5)/Ogre::Math::RangeRandom(1, 5),
                  c->getDerivedPosition().y + Ogre::Math::RangeRandom(
                    -c->getFarClipDistance()*0.5, c->getFarClipDistance()*0.5)/
                  Ogre::Math::RangeRandom(1, 5),
                  mVClouds->getGeometrySettings().Height.x + 0.2*
                  mVClouds->getGeometrySettings().Height.y),
                // Ray direction
                Ogre::Vector3(Ogre::Math::RangeRandom(-1,  1),
                  Ogre::Math::RangeRandom(-1,  1),
                  Ogre::Math::RangeRandom(-0.1,  0.1)).normalisedCopy(),
                // Ray length
                Ogre::Math::RangeRandom(0.5,  1.5f)*0.2*
                mVClouds->getGeometrySettings().Height.y);
          }
          // Cloud-to-ground + cloud-to-cloud
          else
          {
            addLightning(
                // Ray position
                Ogre::Vector3(c->getDerivedPosition().x +
                  Ogre::Math::RangeRandom(-c->getFarClipDistance()*0.5,
                    c->getFarClipDistance()*0.5)/Ogre::Math::RangeRandom(1, 5),
                  c->getDerivedPosition().y + Ogre::Math::RangeRandom(
                    -c->getFarClipDistance()*0.5, c->getFarClipDistance()*0.5)/
                  Ogre::Math::RangeRandom(1,  5),
                  mVClouds->getGeometrySettings().Height.x +
                  0.2*mVClouds->getGeometrySettings().Height.y),
                // Ray direction
                Ogre::Vector3(0,  0,  -1),
                // Ray length
                mVClouds->getGeometrySettings().Height.x +
                0.1*mVClouds->getGeometrySettings().Height.y);

            addLightning(
                // Ray position
                Ogre::Vector3(c->getDerivedPosition().x +
                  Ogre::Math::RangeRandom(-c->getFarClipDistance()*0.5,
                    c->getFarClipDistance()*0.5)/Ogre::Math::RangeRandom(1,  5),
                  c->getDerivedPosition().y + Ogre::Math::RangeRandom(
                    -c->getFarClipDistance()*0.5,  c->getFarClipDistance()*0.5)/
                  Ogre::Math::RangeRandom(1,  5),
                  mVClouds->getGeometrySettings().Height.x +
                  0.2*mVClouds->getGeometrySettings().Height.y),
                // Ray direction
                Ogre::Vector3(Ogre::Math::RangeRandom(-1,  1),
                  Ogre::Math::RangeRandom(-1,  1),
                  Ogre::Math::RangeRandom(-0.1,  0.1)).normalisedCopy(),
                // Ray length
                Ogre::Math::RangeRandom(0.5,  1.5f)*0.2*
                mVClouds->getGeometrySettings().Height.y);
          }

          updateMaterial();
        }
      }
    }

    for (std::vector<Lightning*>::iterator it = mLightnings.begin();
         it != mLightnings.end();)
    {
      if ((*it)->isFinished())
      {
        Ogre::SceneNode* sn = (*it)->getSceneNode();

        delete (*it);
        it = mLightnings.erase(it);

        // Remove the associated scene node
        for (std::vector<Ogre::SceneNode*>::iterator it2 = mSceneNodes.begin();
             it2 != mSceneNodes.end(); it2++)
        {
          if ((*it2) == sn)
          {
            sn->getParentSceneNode()->removeAndDestroyChild(sn->getName());
            mSceneNodes.erase(it2);
            break;
          }
        }
      }
      else
      {
        (*it)->update(timeSinceLastFrame);
        it++;
      }
    }
  }

  Lightning* LightningManager::addLightning(const Ogre::Vector3& p,
      const Ogre::Vector3& d,  const Ogre::Real l,  const Ogre::uint32& div)
  {
    if (!mCreated || mLightnings.size() == 3)
    {
      return static_cast<Lightning*>(NULL);
    }

    Ogre::SceneNode* sn = mVClouds->getSceneManager()->getRootSceneNode()->
      createChildSceneNode();
    sn->setPosition(p);

    Lightning* lightning = new Lightning(mVClouds->getSceneManager(),
        sn,  Ogre::Vector3(0,  0,  0),  d,  l,  div,  3,
        mLightningTimeMultiplier,  mVClouds->getGeometrySettings().Radius/9500);
    lightning->create();
    lightning->_updateRenderQueueGroup(mVClouds->
        getRenderQueueGroups().vcloudsLightnings);
    lightning->getBillboardSet()->setVisible(mVClouds->isVisible());

    mSceneNodes.push_back(sn);
    mLightnings.push_back(lightning);

    for (Ogre::uint32 k = 0; k < mListeners.size(); k++)
    {
      mListeners.at(k)->lightningAdded(lightning);
    }

    return lightning;
  }

  void LightningManager::updateMaterial()
  {
    Ogre::Vector3 pos;

    for (Ogre::uint32 k = 0; k < 3; k++)
    {
      if (k < mLightnings.size())
      {
        pos = mVClouds->getGeometryManager()->getSceneNode()->
          _getFullTransform().inverseAffine() *
          mSceneNodes.at(k)->_getDerivedPosition();

        mVolCloudsLightningMaterial->
          getTechnique(0)->getPass(0)->getFragmentProgramParameters()->
          setNamedConstant("uLightning" + Ogre::StringConverter::toString(k),
              Ogre::Vector4(pos.x,  pos.y,
                pos.z,  mLightnings.at(k)->getIntensity()));
      }
      else
      {
        mVolCloudsLightningMaterial->
          getTechnique(0)->getPass(0)->getFragmentProgramParameters()->
          setNamedConstant("uLightning" + Ogre::StringConverter::toString(k),
              Ogre::Vector4(0,  0,  0,  0));
      }
    }
  }

  void LightningManager::setLightningColor(const Ogre::Vector3& c)
  {
    mLightningColor = c;

    if (!mCreated)
    {
      return;
    }

    mVolCloudsLightningMaterial->getTechnique(0)->getPass(0)->
      getFragmentProgramParameters()
      ->setNamedConstant("uLightningColor",  mLightningColor);

    mLightningMaterial->getTechnique(0)->getPass(0)->
      getFragmentProgramParameters()
      ->setNamedConstant("uColor",  mLightningColor);
  }

  void LightningManager::removeListener(Listener* listener)
  {
    for (std::vector<Listener*>::iterator it = mListeners.begin();
        it != mListeners.end(); it++)
    {
      if ((*it) == listener)
      {
        mListeners.erase(it);
        return;
      }
    }
  }

  void LightningManager::setEnabled(const bool& enable)
  {
    mEnabled = enable;

    if (mCreated)
    {
      if (mEnabled)
      {
        mVClouds->getGeometryManager()->_setMaterialName(
            "SkyX_VolClouds_Lightning");
      }
      else
      {
        mVClouds->getGeometryManager()->_setMaterialName("SkyX_VolClouds");
      }
    }
  }

  void LightningManager::_updateRenderQueueGroup(const Ogre::uint8& /*rqg*/)
  {
    for (Ogre::uint32 k = 0; k < mLightnings.size(); k++)
    {
      mLightnings.at(k)->_updateRenderQueueGroup(mVClouds->
          getRenderQueueGroups().vcloudsLightnings);
    }
  }

  void LightningManager::_setVisible(const bool& v)
  {
    for (Ogre::uint32 k = 0; k < mLightnings.size(); k++)
    {
      mLightnings.at(k)->getBillboardSet()->setVisible(v);
    }
  }
}}
