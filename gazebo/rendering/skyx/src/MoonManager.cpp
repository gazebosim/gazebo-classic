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

#include "MoonManager.h"

#include "SkyX.h"

namespace SkyX
{
  MoonManager::MoonManager(SkyX *s)
    : mSkyX(s)
      , mMoonBillboard(0)
      , mMoonSceneNode(0)
      , mCreated(false)
      , mMoonSize(0.225f)
      , mMoonHaloIntensity(0.4f)
      , mMoonHaloStrength(0.9f)
      , mMoonMaterial(Ogre::MaterialPtr())
      , mVisible(true)
  {
  }

  MoonManager::~MoonManager()
  {
    remove();
  }

  void MoonManager::create()
  {
    if (mCreated)
    {
      return;
    }

    mMoonMaterial = static_cast<Ogre::MaterialPtr>(
        Ogre::MaterialManager::getSingleton().getByName("SkyX_Moon"));

    if (mMoonMaterial.isNull())
    {
      SkyXLOG("Error while creating SkyX::MoonManager, material not found");
      return;
    }

    mMoonSceneNode = mSkyX->getSceneManager()->getRootSceneNode()->
                     createChildSceneNode();

    mMoonBillboard = mSkyX->getSceneManager()->createBillboardSet(
        "SkyXMoonBillboardSet", 1);
    mMoonBillboard->setMaterialName(
        mSkyX->getGPUManager()->getMoonMaterialName());
    mMoonBillboard->setBillboardType(Ogre::BBT_ORIENTED_COMMON);
    mMoonBillboard->setRenderQueueGroup(
        mSkyX->getRenderQueueGroups().skydome+1);
    mMoonBillboard->setCastShadows(false);

    mMoonBillboard->createBillboard(Ogre::Vector3(0, 0, 0));

    mMoonSceneNode->attachObject(mMoonBillboard);

    mCreated = true;
  }

  void MoonManager::remove()
  {
    if (!mCreated)
    {
      return;
    }

    mMoonSceneNode->detachAllObjects();
    mMoonSceneNode->getParentSceneNode()->removeAndDestroyChild(
        mMoonSceneNode->getName());
    mMoonSceneNode = 0;

    mSkyX->getSceneManager()->destroyBillboardSet(mMoonBillboard);
    mMoonBillboard = 0;

    mMoonMaterial.setNull();

    mCreated = false;
  }

  void MoonManager::updateMoonPhase(const Ogre::Real& phase)
  {
    Ogre::Real center = 0, radius = 0, radius_add = 0;
    Ogre::Real interpolation = 0, halo_flip = 0;

    Ogre::Vector3 halo1, halo2;

    // [-1, 0]
    if (phase < 0)
    {
      // [-1, -0.5]
      if (phase < -0.5)
      {
        center = (1+phase)/2;
        radius = 0.25;

        interpolation = center*4;

        if (interpolation < 1.0f/3)
        {
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.25, 0.5, (1-interpolation)*interpolation);
          halo2 = Ogre::Vector3(0.25, 0.5, interpolation);
        }
        else if (interpolation < 2.0f/3)
        {
          interpolation -= 1.0f/3;
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.25, 0.5, 1-interpolation);
          halo2 = Ogre::Vector3(0.0, 0.5, interpolation);
        }
        else
        {
          interpolation -= 2.0f/3;
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.0, 0.5, 1-interpolation);
          halo2 = Ogre::Vector3(0.75, 0.0, interpolation);
        }

        radius_add =  0.1*center/(0.25001-center);

        radius += radius_add;
        center += radius_add;
      }
      // [-0.5, 0]
      else
      {
        center = (-phase)/2;
        radius = 0.25;

        interpolation = 1-center*4;

        if (interpolation < 1.0f/3)
        {
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.75, 0.0, 1-interpolation);
          halo2 = Ogre::Vector3(0.5, 0.0, interpolation);
        }
        else if (interpolation < 2.0f/3)
        {
          interpolation -= 1.0f/3;
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.5, 0.0, 1-interpolation);
          halo2 = Ogre::Vector3(0.25, 0.0, interpolation);
        }
        else
        {
          interpolation -= 2.0f/3;
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.25, 0.0, 1-interpolation);
          halo2 = Ogre::Vector3(0.00, 0.0, interpolation);
        }

        radius_add =  0.1*center/(0.25001-center);

        radius += radius_add;
        center += radius_add;

        radius = -radius;
        center = -center;
      }
    }
    // [0, 1]
    else
    {
      halo_flip = 1;

      // [0, 0.5]
      if (phase < 0.5)
      {
        center = phase/2;
        radius = 0.25;

        interpolation = center*4;

        if (interpolation < 1.0f/3)
        {
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.00, 0.0, 1-interpolation);
          halo2 = Ogre::Vector3(0.25, 0.0, interpolation);
        }
        else if (interpolation < 2.0f/3)
        {
          interpolation -= 1.0f/3;
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.25, 0.0, 1-interpolation);
          halo2 = Ogre::Vector3(0.5, 0.0, interpolation);
        }
        else
        {
          interpolation -= 2.0f/3;
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.5, 0.0, 1-interpolation);
          halo2 = Ogre::Vector3(0.75, 0.0, interpolation);
        }

        radius_add =  0.1*center/(0.25001-center);

        radius += radius_add;
        center += radius_add;

        radius = -radius;
        // clang warning: explicitly assigning value of variable of type
        // 'Ogre::Real' (aka 'float') to itself
        //
        // center = center;
      }
      // [0.5, 1]
      else
      {
        center = (1-phase)/2;
        radius = 0.25;

        interpolation = 1-center*4;

        if (interpolation < 1.0f/3)
        {
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.75, 0.0, 1-interpolation);
          halo2 = Ogre::Vector3(0.0, 0.5, interpolation);
        }
        else if (interpolation < 2.0f/3)
        {
          interpolation -= 1.0f/3;
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.0, 0.5, 1-interpolation);
          halo2 = Ogre::Vector3(0.25, 0.5, interpolation);
        }
        else
        {
          interpolation -= 2.0f/3;
          interpolation /= 1.0f/3;
          halo1 = Ogre::Vector3(0.25, 0.5, 1-interpolation);
          halo2 = Ogre::Vector3(0.25, 0.5, (1-interpolation)*interpolation);
        }

        radius_add =  0.1*center/(0.25001-center);

        radius += radius_add;
        center += radius_add;

        center = -center;
      }
    }

    mMoonMaterial->getTechnique(0)->getPass(0)
      ->getFragmentProgramParameters()->setNamedConstant(
          "uMoonPhase", Ogre::Vector3(radius, center + 0.5f,
            mMoonHaloStrength));

    halo1.z *= mMoonHaloIntensity;
    halo2.z *= mMoonHaloIntensity;

    mMoonMaterial->getTechnique(0)->getPass(0)
      ->getFragmentProgramParameters()->setNamedConstant("uMoonHalo1", halo1);
    mMoonMaterial->getTechnique(0)->getPass(0)
      ->getFragmentProgramParameters()->setNamedConstant("uMoonHalo2", halo2);
    mMoonMaterial->getTechnique(0)->getPass(0)
      ->getFragmentProgramParameters()->setNamedConstant("uMoonHaloFlip",
          halo_flip);
  }

  void MoonManager::updateGeometry(Ogre::Camera* c)
  {
    if (!mCreated)
    {
      return;
    }

    float radius = mSkyX->getMeshManager()->getSkydomeRadius(c)*0.95f,
          size = radius*mMoonSize;

    mMoonBillboard->setCommonDirection((mSkyX->getController()->
          getMoonDirection()).normalisedCopy().perpendicular());

    Ogre::Vector3 moonRelativePos = mSkyX->getController()->
      getMoonDirection()*
      Ogre::Math::Cos(Ogre::Math::ASin((size/2)/radius))*radius;

    mMoonSceneNode->setPosition(c->getDerivedPosition() + moonRelativePos);

    if (moonRelativePos.z < -size/2)
    {
      mMoonSceneNode->setVisible(false);
    }
    else
    {
      mMoonSceneNode->setVisible(mSkyX->isVisible());

      mMoonMaterial->getTechnique(0)->getPass(0)
        ->getVertexProgramParameters()->setNamedConstant(
            "uSkydomeCenter", c->getDerivedPosition());
    }

    if (!equal(mMoonBillboard->getBoundingBox().getMaximum().x, size))
    {
      _updateMoonBounds(c);
    }
  }

  void MoonManager::_updateMoonBounds(Ogre::Camera* c)
  {
    float radius = mSkyX->getMeshManager()->getSkydomeRadius(c)*0.95f,
          size = radius*mMoonSize;

    mMoonBillboard->setDefaultDimensions(size, size);
    mMoonBillboard->setBounds(Ogre::AxisAlignedBox(-size/2, -size/2, -size/2,
          size/2,  size/2,  size/2), 1);
    mMoonSceneNode->_updateBounds();
  }
}
