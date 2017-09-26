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
#ifndef _CUSTOMPSSMSHADOWCAMERASETUP_HH_
#define _CUSTOMPSSMSHADOWCAMERASETUP_HH_

#include <OgreShadowCameraSetupPSSM.h>

namespace Ogre
{
	/** Parallel Split Shadow Map (PSSM) shadow camera setup. 
	@remarks
    Ogre's LiSPSM algorithm makes buggy shadow frusta that often put high
    resolution areas in the wrong places. To fix this we subclass a new
    ShadowCameraSetup class that does not use LiSPSM.
    The solution to this problem in Ogre 2.1 was to derive from
    FocusedShadowCameraSetup instead of LiSPSMShadowCameraSetup.
    Instead, we are using a modified version of Focused shadow maps. Modified
    because we want the light's shadow frusta to always be z-up just like
    gazebo. The original Focused shadows in Ogre are y-up. Aligning shadow map
    frusta with the up direction tends to minimize shadow map stretching
    artifacts by letting stretched shadow texels form long rectangles instead
    instead of sawtooth patterns on most surfaces such as terrain and man-made
    objects.
	*/
  class _OgreExport CustomPSSMShadowCameraSetup : public PSSMShadowCameraSetup
	{
	public:
		/// Constructor, defaults to 3 splits
		CustomPSSMShadowCameraSetup();
		~CustomPSSMShadowCameraSetup();

    /** Slightly modified FocusedShadowCameraSetup::calculateShadowMappingMatrix().
     */
    void calculateShadowMappingMatrix(const SceneManager& sm,
      const Camera& cam, const Light& light, Matrix4 *out_view, Matrix4 *out_proj,
      Camera *out_cam) const;

    /** The same as FocusedShadowCameraSetup::buildViewMatrix() except resulting
     * matrices are z-up instead of y-up.
     */
    Matrix4 buildViewMatrix(const Vector3& pos, const Vector3& dir, const Vector3& up) const;

    /** The same as FocusedShadowCameraSetup::ShadowCameraSetup() except
     * resulting light frusta are z-up instead of y-up.
     */
    virtual void getZUpFocusedShadowCamera(const SceneManager *sm, const Camera *cam,
      const Viewport *vp, const Light *light, Camera *texCam, size_t iteration) const;

    /// Returns a shadow camera with PSSM splits based on iteration.
    virtual void getShadowCamera(const SceneManager *sm, const Camera *cam,
      const Viewport *vp, const Light *light, Camera *texCam, size_t iteration) const;
	};
}

#endif
