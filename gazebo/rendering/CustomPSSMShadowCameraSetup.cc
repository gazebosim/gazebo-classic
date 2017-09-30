/*
-----------------------------------------------------------------------------
This source file is part of OGRE
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2014 Torus Knot Software Ltd
Copyright (c) 2006 Matthias Fink, netAllied GmbH <matthias.fink@web.de>

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
#include "CustomPSSMShadowCameraSetup.hh"
#include "gazebo/rendering/ogre_gazebo.h"

namespace Ogre
{
	//---------------------------------------------------------------------
	CustomPSSMShadowCameraSetup::CustomPSSMShadowCameraSetup()
	{
	}
	//---------------------------------------------------------------------
	CustomPSSMShadowCameraSetup::~CustomPSSMShadowCameraSetup()
	{
  }

  //-----------------------------------------------------------------------
  void CustomPSSMShadowCameraSetup::calculateShadowMappingMatrix(const SceneManager& sm,
    const Camera& cam, const Light& light, Matrix4 *out_view, Matrix4 *out_proj,
    Camera *out_cam) const
  {
    // get the shadow frustum's far distance
    Real shadowDist = light.getShadowFarDistance();
    if (!shadowDist)
    {
      // need a shadow distance, make one up
      shadowDist = cam.getNearClipDistance() * 3000;
    }
    Real shadowOffset = shadowDist * sm.getShadowDirLightTextureOffset();


    if (light.getType() == Light::LT_DIRECTIONAL)
    {
      // generate view matrix if requested
      if (out_view != NULL)
      {
        Vector3 pos;
        if (sm.getCameraRelativeRendering())
        {
          pos = Vector3::ZERO;
        }
        else
        {
          pos = cam.getDerivedPosition();
        }
        *out_view = buildViewMatrix(pos,
          light.getDerivedDirection(),
          //cam.getDerivedUp());
          Vector3::UNIT_Z); // Modified for z-up light frusta
      }

      // generate projection matrix if requested
      if (out_proj != NULL)
      {
        *out_proj = Matrix4::getScale(1, 1, -1);
        //*out_proj = Matrix4::IDENTITY;
      }

      // set up camera if requested
      if (out_cam != NULL)
      {
        out_cam->setProjectionType(PT_ORTHOGRAPHIC);
        out_cam->setDirection(light.getDerivedDirection());
        out_cam->setPosition(cam.getDerivedPosition());
        out_cam->setFOVy(Degree(90));
        out_cam->setNearClipDistance(shadowOffset);
      }
    }
    else if (light.getType() == Light::LT_POINT)
    {
      // target analogue to the default shadow textures
      // Calculate look at position
      // We want to look at a spot shadowOffset away from near plane
      // 0.5 is a little too close for angles
      Vector3 target = cam.getDerivedPosition() +
        (cam.getDerivedDirection() * shadowOffset);
      Vector3 lightDir = target - light.getDerivedPosition();
      lightDir.normalise();

      // generate view matrix if requested
      if (out_view != NULL)
      {
        *out_view = buildViewMatrix(light.getDerivedPosition(),
          lightDir,
          cam.getDerivedUp());
      }

      // generate projection matrix if requested
      if (out_proj != NULL)
      {
        // set FOV to 120 degrees
        mTempFrustum->setFOVy(Degree(120));

        mTempFrustum->setNearClipDistance(light._deriveShadowNearClipDistance(&cam));
        mTempFrustum->setFarClipDistance(light._deriveShadowFarClipDistance(&cam));

        *out_proj = mTempFrustum->getProjectionMatrix();
      }

      // set up camera if requested
      if (out_cam != NULL)
      {
        out_cam->setProjectionType(PT_PERSPECTIVE);
        out_cam->setDirection(lightDir);
        out_cam->setPosition(light.getDerivedPosition());
        out_cam->setFOVy(Degree(120));
        out_cam->setNearClipDistance(light._deriveShadowNearClipDistance(&cam));
        out_cam->setFarClipDistance(light._deriveShadowFarClipDistance(&cam));
      }
    }
    else if (light.getType() == Light::LT_SPOTLIGHT)
    {
      // generate view matrix if requested
      if (out_view != NULL)
      {
        *out_view = buildViewMatrix(light.getDerivedPosition(),
          light.getDerivedDirection(),
          cam.getDerivedUp());
      }

      // generate projection matrix if requested
      if (out_proj != NULL)
      {
        // set FOV slightly larger than spotlight range
        mTempFrustum->setFOVy(Ogre::Math::Clamp<Radian>(light.getSpotlightOuterAngle() * 1.2, Radian(0), Radian(Math::PI/2.0f)));

        mTempFrustum->setNearClipDistance(light._deriveShadowNearClipDistance(&cam));
        mTempFrustum->setFarClipDistance(light._deriveShadowFarClipDistance(&cam));

        *out_proj = mTempFrustum->getProjectionMatrix();
      }

      // set up camera if requested
      if (out_cam != NULL)
      {
        out_cam->setProjectionType(PT_PERSPECTIVE);
        out_cam->setDirection(light.getDerivedDirection());
        out_cam->setPosition(light.getDerivedPosition());
        out_cam->setFOVy(Ogre::Math::Clamp<Radian>(light.getSpotlightOuterAngle() * 1.2, Radian(0), Radian(Math::PI/2.0f)));
        out_cam->setNearClipDistance(light._deriveShadowNearClipDistance(&cam));
        out_cam->setFarClipDistance(light._deriveShadowFarClipDistance(&cam));
      }
    }
  }

  //-----------------------------------------------------------------------
  Matrix4 CustomPSSMShadowCameraSetup::buildViewMatrix(const Vector3& pos, const Vector3& dir,
      const Vector3& up) const
    {
      Vector3 xN = dir.crossProduct(up);
      xN.normalise();
      Vector3 upN = xN.crossProduct(dir);
      upN.normalise();

      // Modified for z-up light frusta
      Matrix4 m(xN.x,		xN.y,		xN.z,		-xN.dotProduct(pos),
        dir.x,		dir.y,	dir.z,	-dir.dotProduct(pos),
        upN.x,		upN.y,		upN.z,		-upN.dotProduct(pos),
        //-dir.x,		-dir.y,	-dir.z,	dir.dotProduct(pos),
        0.0,			0.0,		0.0,		1.0
        );

      return m;
    }

  //-----------------------------------------------------------------------
  void CustomPSSMShadowCameraSetup::getZUpFocusedShadowCamera (const SceneManager *sm, const Camera *cam,
    const Viewport *vp, const Light *light, Camera *texCam, size_t iteration) const
  {
    // check availability - viewport not needed
    OgreAssert(sm != NULL, "SceneManager is NULL");
    OgreAssert(cam != NULL, "Camera (viewer) is NULL");
    OgreAssert(light != NULL, "Light is NULL");
    OgreAssert(texCam != NULL, "Camera (texture) is NULL");
    mLightFrustumCameraCalculated = false;

    texCam->setNearClipDistance(light->_deriveShadowNearClipDistance(cam));
    texCam->setFarClipDistance(light->_deriveShadowFarClipDistance(cam));

    // calculate standard shadow mapping matrix
    Matrix4 LView, LProj;
    calculateShadowMappingMatrix(*sm, *cam, *light, &LView, &LProj, NULL);

    // build scene bounding box
    const VisibleObjectsBoundsInfo& visInfo = sm->getVisibleObjectsBoundsInfo(texCam);
    AxisAlignedBox sceneBB = visInfo.aabb;
    AxisAlignedBox receiverAABB = sm->getVisibleObjectsBoundsInfo(cam).receiverAabb;
    sceneBB.merge(receiverAABB);
    sceneBB.merge(cam->getDerivedPosition());

    // in case the sceneBB is empty (e.g. nothing visible to the cam) simply
    // return the standard shadow mapping matrix
    if (sceneBB.isNull())
    {
      texCam->setCustomViewMatrix(true, LView);
      texCam->setCustomProjectionMatrix(true, LProj);
      return;
    }

    // calculate the intersection body B
    mPointListBodyB.reset();
    calculateB(*sm, *cam, *light, sceneBB, receiverAABB, &mPointListBodyB);

    // in case the bodyB is empty (e.g. nothing visible to the light or the cam)
    // simply return the standard shadow mapping matrix
    if (mPointListBodyB.getPointCount() == 0)
    {
      texCam->setCustomViewMatrix(true, LView);
      texCam->setCustomProjectionMatrix(true, LProj);
      return;
    }

    // transform to light space: y -> -z, z -> y
    LProj = msNormalToLightSpace * LProj;

    // calculate LVS so it does not need to be calculated twice
    // calculate the body L \cap V \cap S to make sure all returned points are in
    // front of the camera
    //mPointListBodyLVS.reset();
    //calculateLVS(*sm, *cam, *light, sceneBB, &mPointListBodyLVS);

    // fetch the viewing direction
    //const Vector3 viewDir = getLSProjViewDir(LProj * LView, *cam, mPointListBodyLVS);

    // The light space will be rotated in such a way, that the projected light view
    // always points upwards, so the up-vector is the y-axis (we already prepared the
    // light space for this usage).The transformation matrix is set up with the
    // following parameters:
    // - position is the origin
    // - the view direction is the calculated viewDir
    // - the up vector is the y-axis
    // Commented out for z-up light frusta
    //LProj = buildViewMatrix(Vector3::ZERO, viewDir, Vector3::UNIT_Y) * LProj;

    // map bodyB to unit cube
    LProj = transformToUnitCube(LProj * LView, mPointListBodyB) * LProj;

    // transform from light space to normal space: y -> z, z -> -y
    // Commented out for z-up light frusta
    //LProj = msLightSpaceToNormal * LProj;

    // set the two custom matrices
    texCam->setCustomViewMatrix(true, LView);
    texCam->setCustomProjectionMatrix(true, LProj);
  }

	//---------------------------------------------------------------------
  void CustomPSSMShadowCameraSetup::getShadowCamera(const SceneManager *sm, const Camera *cam,
    const Viewport *vp, const Light *light, Camera *texCam, size_t iteration) const
	{
		// apply the right clip distance.
		Real nearDist = mSplitPoints[iteration];
		Real farDist = mSplitPoints[iteration + 1];

		// Add a padding factor to internal distances so that the connecting split point will not have bad artifacts.
		if (iteration > 0)
		{
      nearDist -= mSplitPadding;
      nearDist = std::max( nearDist, mSplitPoints[0] );
		}
		if (iteration < mSplitCount - 1)
		{
			farDist += mSplitPadding;
		}

		mCurrentIteration = iteration;

		// Ouch, I know this is hacky, but it's the easiest way to re-use LiSPSM / Focused
		// functionality right now without major changes
		Camera* _cam = const_cast<Camera*>(cam);
		Real oldNear = _cam->getNearClipDistance();
    Real oldFar = _cam->getFarClipDistance();
		_cam->setNearClipDistance(nearDist);
		_cam->setFarClipDistance(farDist);

    // Replaced LiSPSMShadowCameraSetup::getShadowCamera() with
    // FocusedShadowCameraSetup::getShadowCamera(). This is the same solution
    // they reached for Ogre 2.1 after realizing how buggy LiSPSM is.
    // However, unlike Ogre 2.1, we are using a modified method that makes a
    // shadow camera frustum aligned with the z-axis because Gazebo is z-up.
    //LiSPSMShadowCameraSetup::getShadowCamera(sm, cam, vp, light, texCam, iteration);
    //FocusedShadowCameraSetup::getShadowCamera(sm, cam, vp, light, texCam, iteration);
    getZUpFocusedShadowCamera(sm, cam, vp, light, texCam, iteration);

		// restore near/far
		_cam->setNearClipDistance(oldNear);
		_cam->setFarClipDistance(oldFar);
  }
}
