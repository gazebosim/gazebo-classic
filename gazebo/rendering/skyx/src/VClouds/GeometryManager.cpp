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
#include "VClouds/GeometryManager.h"

#include "VClouds/VClouds.h"

namespace SkyX { namespace VClouds
{
    GeometryManager::GeometryManager(VClouds* vc)
        : mVClouds(vc)
        , mCreated(false)
        , mHeight(Ogre::Vector2())
        , mRadius(0)
        , mAlpha(0)
        , mBeta(0)
        , mPhi(0)
        , mNumberOfBlocks(0)
        , mNa(0), mNb(0), mNc(0)
        , mA(0), mB(0), mC(0)
        , mWorldOffset(Ogre::Vector2(0, 0))
    {
    }

    GeometryManager::~GeometryManager()
    {
        remove();
    }

    void GeometryManager::create(const Ogre::Vector2& Height,
            const float& Radius,
            const Ogre::Radian& Alpha, const Ogre::Radian& Beta,
            const int& NumberOfBlocks, const int& Na, const int& Nb,
            const int& Nc)
    {
        remove();

        mHeight = Height;
        mRadius = Radius;
        mAlpha = Alpha;
        mBeta = Beta;
        mPhi = Ogre::Math::TWO_PI / NumberOfBlocks;
        mNumberOfBlocks = NumberOfBlocks;
        mNa = Na;
        mNb = Nb;
        mNc = Nc;

        mSceneNode = mVClouds->getSceneManager()->getRootSceneNode()->
          createChildSceneNode();
        _createGeometry();

        mCreated = true;
    }

    void GeometryManager::remove()
    {
        if (!mCreated)
        {
            return;
        }

        mSceneNode->detachAllObjects();
        mSceneNode->getParentSceneNode()->removeAndDestroyChild(
            mSceneNode->getName());
        mSceneNode = 0;

        for (int k = 0; k < mNumberOfBlocks; k++)
        {
            delete mGeometryBlocks.at(k);
            mGeometryBlocks.at(k) = 0;
        }

        mGeometryBlocks.clear();

        mCreated = false;
    }

    void GeometryManager::update(const Ogre::Real& timeSinceLastFrame)
    {
        if (!mCreated)
        {
            return;
        }

        mWorldOffset += mVClouds->getWindDirectionV2() *
          mVClouds->getWindSpeed() * timeSinceLastFrame;
    }

    void GeometryManager::updateGeometry(Ogre::Camera* c,
        const Ogre::Real& timeSinceLastCameraFrame)
    {
        if (!mCreated)
        {
            return;
        }

        mSceneNode->setPosition(mVClouds->getCamera()->getDerivedPosition().x,
            mVClouds->getCamera()->getDerivedPosition().y, mHeight.x);
        mSceneNode->_update(false, false);

        _updateGeometry(c, timeSinceLastCameraFrame);
    }

    void GeometryManager::_setMaterialName(const Ogre::String& mn)
    {
        for (Ogre::uint32 k = 0; k < mGeometryBlocks.size(); k++)
        {
            mGeometryBlocks.at(k)->getEntity()->setMaterialName(mn);
        }
    }

    void GeometryManager::_updateRenderQueueGroup(const Ogre::uint8& rqg)
    {
        for (Ogre::uint32 k = 0; k < mGeometryBlocks.size(); k++)
        {
            mGeometryBlocks.at(k)->getEntity()->setRenderQueueGroup(rqg);
        }
    }

    void GeometryManager::_createGeometry()
    {
        mA = mHeight.y /
          Ogre::Math::Cos(Ogre::Math::PI/2-mBeta.valueRadians());
        mB = mHeight.y /
          Ogre::Math::Cos(Ogre::Math::PI/2-mAlpha.valueRadians());
        mC = mRadius;

        for (int k = 0; k < mNumberOfBlocks; k++)
        {
            mGeometryBlocks.push_back(new GeometryBlock(mVClouds,
                  mHeight.y, mAlpha, mBeta, mRadius, mPhi, mNa, mNb, mNc,
                  mA, mB, mC, k));
            mGeometryBlocks.at(k)->create();
            // Each geometry block must be in a different scene node,
            // See: GeometryBlock::isInFrustum(Ogre::Camera *c)
            Ogre::SceneNode *sn = mSceneNode->createChildSceneNode();
            sn->attachObject(mGeometryBlocks.at(k)->getEntity());
        }
    }

    void GeometryManager::_updateGeometry(Ogre::Camera* c,
        const Ogre::Real& timeSinceLastFrame)
    {
        // Look for current camera data
        std::vector<VClouds::CameraData>& camerasData =
          mVClouds->_getCamerasData();
        std::vector<VClouds::CameraData>::iterator currentCameraDataIt;

        for (currentCameraDataIt = camerasData.begin();
            currentCameraDataIt != camerasData.end(); currentCameraDataIt++)
        {
            if ((*currentCameraDataIt).camera == c)
            {
                break;
            }
        }

        std::vector<VClouds::CameraData>::reference currentCameraData =
          (*currentCameraDataIt);

        // Calculate wind offset
        Ogre::Vector2 CameraDirection =
          Ogre::Vector2(c->getDerivedDirection().x, c->getDerivedDirection().y);
        float offset = -CameraDirection.dotProduct(
            mVClouds->getWindDirectionV2()) * mVClouds->getWindSpeed() *
          timeSinceLastFrame;

        // Calculate camera offset
        Ogre::Vector2 CameraOffset = Ogre::Vector2(
            c->getDerivedPosition().x - currentCameraData.lastPosition.x,
            c->getDerivedPosition().y - currentCameraData.lastPosition.y);
        offset -= CameraOffset.dotProduct(CameraDirection);

        // Update camera data
        currentCameraData.cameraOffset += CameraOffset;
        currentCameraData.lastPosition = c->getDerivedPosition();

        // Update geometry displacement
        currentCameraData.geometryDisplacement += Ogre::Vector3(offset);

        if (currentCameraData.geometryDisplacement.z < 0 ||
            currentCameraData.geometryDisplacement.z > (mC-mB)/mNc)
        {
            currentCameraData.geometryDisplacement.z -=
              ((mC-mB)/mNc)*Ogre::Math::IFloor(
                (currentCameraData.geometryDisplacement.z)/((mC-mB)/mNc));
        }

        if (currentCameraData.geometryDisplacement.y < 0 ||
            currentCameraData.geometryDisplacement.y > (mB-mA)/mNb)
        {
            currentCameraData.geometryDisplacement.y -=
              ((mB-mA)/mNb)*Ogre::Math::IFloor(
                (currentCameraData.geometryDisplacement.y)/((mB-mA)/mNb));
        }

        if (currentCameraData.geometryDisplacement.x < 0 ||
            currentCameraData.geometryDisplacement.x > mA/mNa)
        {
            currentCameraData.geometryDisplacement.x -=
              (mA/mNa)*Ogre::Math::IFloor(
                  (currentCameraData.geometryDisplacement.x)/(mA/mNa));
        }

        for (int k = 0; k < mNumberOfBlocks; k++)
        {
            mGeometryBlocks.at(k)->setWorldOffset(
                mWorldOffset + currentCameraData.cameraOffset);
            mGeometryBlocks.at(k)->updateGeometry(c,
                currentCameraData.geometryDisplacement);
        }
    }
}}
