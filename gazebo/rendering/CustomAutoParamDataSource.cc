/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

// Code in this file has been adapted from Ogre's AutoParamDataSource. The
// original Ogre licence and copyright headers are copied below:

/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2014 Torus Knot Software Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include "gazebo/rendering/CustomAutoParamDataSource.hh"

using namespace gazebo;
using namespace rendering;
using namespace Ogre;

const Matrix4 PROJECTIONCLIPSPACE2DTOIMAGESPACE_PERSPECTIVE(
    0.5,    0,    0,  0.5,
    0,   -0.5,    0,  0.5,
    0,      0,    1,    0,
    0,      0,    0,    1);

void CustomAutoParamDataSource::setCurrentRenderable(const Renderable* rend)
{
  mCurrentRenderable = rend;
  mWorldMatrixDirty = true;
  mViewMatrixDirty = true;
  mProjMatrixDirty = true;
  mWorldViewMatrixDirty = true;
  mViewProjMatrixDirty = true;
  mWorldViewProjMatrixDirty = true;
  mInverseWorldMatrixDirty = true;
  mInverseViewMatrixDirty = true;
  mInverseWorldViewMatrixDirty = true;
  mInverseTransposeWorldMatrixDirty = true;
  mInverseTransposeWorldViewMatrixDirty = true;
  mCameraPositionObjectSpaceDirty = true;
  mLodCameraPositionObjectSpaceDirty = true;
  for(size_t i = 0; i < mTextureWorldViewProjMatrixDirtyVector.size(); ++i)
  {
    mTextureWorldViewProjMatrixDirtyVector[i] = true;
    mSpotlightWorldViewProjMatrixDirtyVector[i] = true;
  }
}

void CustomAutoParamDataSource::setCurrentLightList(const LightList* ll)
{
  resizeVectorsIfNecessary(ll->size());

  mCurrentLightList = ll;
  for(size_t i = 0; i < ll->size(); ++i)
  {
    mSpotlightViewProjMatrixDirtyVector[i] = true;
    mSpotlightWorldViewProjMatrixDirtyVector[i] = true;
  }
}

void CustomAutoParamDataSource::setTextureProjector(const Frustum* frust, size_t index = 0)
{
  resizeVectorsIfNecessary(index + 1);

  mCurrentTextureProjectorVector[index] = frust;
  mTextureViewProjMatrixDirtyVector[index] = true;
  mTextureWorldViewProjMatrixDirtyVector[index] = true;
  mShadowCamDepthRangesDirtyVector[index] = true;
}

const Matrix4& CustomAutoParamDataSource::getTextureViewProjMatrix(size_t index) const
{
  resizeVectorsIfNecessary(index + 1);

  if (mTextureViewProjMatrixDirtyVector[index] && mCurrentTextureProjectorVector[index])
  {
    if (mCameraRelativeRendering)
    {
      // World positions are now going to be relative to the camera position
      // so we need to alter the projector view matrix to compensate
      Ogre::Matrix4 viewMatrix;
      mCurrentTextureProjectorVector[index]->calcViewMatrixRelative(
        mCurrentCamera->getDerivedPosition(), viewMatrix);
      mTextureViewProjMatrixVector[index] =
        PROJECTIONCLIPSPACE2DTOIMAGESPACE_PERSPECTIVE *
        mCurrentTextureProjectorVector[index]->getProjectionMatrixWithRSDepth() *
        viewMatrix;
    }
    else
    {
      mTextureViewProjMatrixVector[index] =
        PROJECTIONCLIPSPACE2DTOIMAGESPACE_PERSPECTIVE *
        mCurrentTextureProjectorVector[index]->getProjectionMatrixWithRSDepth() *
        mCurrentTextureProjectorVector[index]->getViewMatrix();
    }
    mTextureViewProjMatrixDirtyVector[index] = false;
  }
  return mTextureViewProjMatrixVector[index];
}

const Matrix4& CustomAutoParamDataSource::getTextureWorldViewProjMatrix(size_t index) const
{
  resizeVectorsIfNecessary(index + 1);

  if (mTextureWorldViewProjMatrixDirtyVector[index] && mCurrentTextureProjectorVector[index])
  {
    mTextureWorldViewProjMatrixVector[index] =
      getTextureViewProjMatrix(index) * getWorldMatrix();
    mTextureWorldViewProjMatrixDirtyVector[index] = false;
  }
  return mTextureWorldViewProjMatrixVector[index];
}

const Matrix4& CustomAutoParamDataSource::getSpotlightViewProjMatrix(size_t index) const
{
  resizeVectorsIfNecessary(index + 1);

  const Light& l = getLight(index);

  if (&l != &mBlankLight &&
    l.getType() == Light::LT_SPOTLIGHT &&
    mSpotlightViewProjMatrixDirtyVector[index])
  {
    Frustum frust;
    SceneNode dummyNode(0);
    dummyNode.attachObject(&frust);

    frust.setProjectionType(PT_PERSPECTIVE);
    frust.setFOVy(l.getSpotlightOuterAngle());
    frust.setAspectRatio(1.0f);
    // set near clip the same as main camera, since they are likely
    // to both reflect the nature of the scene
    frust.setNearClipDistance(mCurrentCamera->getNearClipDistance());
    // Calculate position, which same as spotlight position, in camera-relative coords if required
    dummyNode.setPosition(l.getDerivedPosition(true));
    // Calculate direction, which same as spotlight direction
    Vector3 dir = - l.getDerivedDirection(); // backwards since point down -z
    dir.normalise();
    Vector3 up = Vector3::UNIT_Y;
    // Check it's not coincident with dir
    if (Math::Abs(up.dotProduct(dir)) >= 1.0f)
    {
      // Use camera up
      up = Vector3::UNIT_Z;
    }
    // cross twice to rederive, only direction is unaltered
    Vector3 left = dir.crossProduct(up);
    left.normalise();
    up = dir.crossProduct(left);
    up.normalise();
    // Derive quaternion from axes
    Quaternion q;
    q.FromAxes(left, up, dir);
    dummyNode.setOrientation(q);

    // The view matrix here already includes camera-relative changes if necessary
    // since they are built into the frustum position
    mSpotlightViewProjMatrixVector[index] =
      PROJECTIONCLIPSPACE2DTOIMAGESPACE_PERSPECTIVE *
      frust.getProjectionMatrixWithRSDepth() *
      frust.getViewMatrix();

    mSpotlightViewProjMatrixDirtyVector[index] = false;
  }
  return mSpotlightViewProjMatrixVector[index];
}

const Matrix4& CustomAutoParamDataSource::getSpotlightWorldViewProjMatrix(size_t index) const
{
  resizeVectorsIfNecessary(index + 1);

  const Light& l = getLight(index);

  if (&l != &mBlankLight &&
    l.getType() == Light::LT_SPOTLIGHT &&
    mSpotlightWorldViewProjMatrixDirtyVector[index])
  {
    mSpotlightWorldViewProjMatrixVector[index] =
      getSpotlightViewProjMatrix(index) * getWorldMatrix();
    mSpotlightWorldViewProjMatrixDirtyVector[index] = false;
  }
  return mSpotlightWorldViewProjMatrixVector[index];
}

const Vector4& CustomAutoParamDataSource::getShadowSceneDepthRange(size_t index) const
{
  static Vector4 dummy(0, 100000, 100000, 1/100000);

  if (!mCurrentSceneManager->isShadowTechniqueTextureBased())
    return dummy;

  resizeVectorsIfNecessary(index + 1);

  if (mShadowCamDepthRangesDirtyVector[index] && mCurrentTextureProjectorVector[index])
  {
    const VisibleObjectsBoundsInfo& info =
      mCurrentSceneManager->getVisibleObjectsBoundsInfo(
        (const Camera*)mCurrentTextureProjectorVector[index]);

    Real depthRange = info.maxDistanceInFrustum - info.minDistanceInFrustum;
    if (depthRange > std::numeric_limits<Real>::epsilon())
    {
      mShadowCamDepthRangesVector[index] = Vector4(
        info.minDistanceInFrustum,
        info.maxDistanceInFrustum,
        depthRange,
        1.0f / depthRange);
    }
    else
    {
      mShadowCamDepthRangesVector[index] = dummy;
    }

    mShadowCamDepthRangesDirtyVector[index] = false;
  }
  return mShadowCamDepthRangesVector[index];
}

void CustomAutoParamDataSource::resizeVectorsIfNecessary(size_t size) const
{
  const size_t oldSize = mTextureViewProjMatrixVector.size();

  // Abort if vectors are already big enough
  if (oldSize >= size)
    return;

  // Resize vectors
  mTextureViewProjMatrixVector.resize(size);
  mTextureWorldViewProjMatrixVector.resize(size);
  mSpotlightViewProjMatrixVector.resize(size);
  mSpotlightWorldViewProjMatrixVector.resize(size);
  mShadowCamDepthRangesVector.resize(size);
  mTextureViewProjMatrixDirtyVector.resize(size);
  mTextureWorldViewProjMatrixDirtyVector.resize(size);
  mSpotlightViewProjMatrixDirtyVector.resize(size);
  mSpotlightWorldViewProjMatrixDirtyVector.resize(size);
  mShadowCamDepthRangesDirtyVector.resize(size);
  mCurrentTextureProjectorVector.resize(size);

  // Initialize new elements of vectors
  for (size_t i = oldSize; i < size; i ++) {
    mTextureViewProjMatrixDirtyVector[i] = true;
    mTextureWorldViewProjMatrixDirtyVector[i] = true;
    mSpotlightViewProjMatrixDirtyVector[i] = true;
    mSpotlightWorldViewProjMatrixDirtyVector[i] = true;
    mShadowCamDepthRangesDirtyVector[i] = true;
    mCurrentTextureProjectorVector[i] = nullptr;
  }
}
