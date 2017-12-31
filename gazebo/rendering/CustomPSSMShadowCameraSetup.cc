/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

// Code in this file has been adapted from Ogre's RTShader::IntegratedPSSM3,
// and different ShadowCameraSetup classes. The original Ogre's licence and
// copyright headers are copied below:

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

#include "gazebo/rendering/CustomPSSMShadowCameraSetup.hh"
#include "gazebo/rendering/ogre_gazebo.h"

using namespace gazebo;
using namespace rendering;

Ogre::String CustomPSSM3::Type = "CustomPSSM3";

//////////////////////////////////////////////////
const Ogre::String &CustomPSSM3::getType() const
{
  return CustomPSSM3::Type;
}

//////////////////////////////////////////////////
bool CustomPSSM3::resolveParameters(Ogre::RTShader::ProgramSet *_programSet)
{
  Ogre::RTShader::Program* vsProgram = _programSet->getCpuVertexProgram();
  Ogre::RTShader::Program* psProgram = _programSet->getCpuFragmentProgram();
  Ogre::RTShader::Function* vsMain = vsProgram->getEntryPointFunction();
  Ogre::RTShader::Function* psMain = psProgram->getEntryPointFunction();

  // Get input position parameter.
  mVSInPos = vsMain->getParameterBySemantic(vsMain->getInputParameters(),
      Ogre::RTShader::Parameter::SPS_POSITION, 0);

  // Get output position parameter.
  mVSOutPos = vsMain->getParameterBySemantic(vsMain->getOutputParameters(),
      Ogre::RTShader::Parameter::SPS_POSITION, 0);

  // Resolve vertex shader output depth.
  mVSOutDepth = vsMain->resolveOutputParameter(
      Ogre::RTShader::Parameter::SPS_TEXTURE_COORDINATES, -1,
      Ogre::RTShader::Parameter::SPC_DEPTH_VIEW_SPACE,
      Ogre::GCT_FLOAT1);

  // Resolve input depth parameter.
  mPSInDepth = psMain->resolveInputParameter(
      Ogre::RTShader::Parameter::SPS_TEXTURE_COORDINATES,
      mVSOutDepth->getIndex(),
      mVSOutDepth->getContent(),
      Ogre::GCT_FLOAT1);

  // Get in/local diffuse parameter.
  mPSDiffuse = psMain->getParameterBySemantic(psMain->getInputParameters(),
      Ogre::RTShader::Parameter::SPS_COLOR, 0);
  if (mPSDiffuse.get() == NULL)
  {
    mPSDiffuse = psMain->getParameterBySemantic(
        psMain->getLocalParameters(), Ogre::RTShader::Parameter::SPS_COLOR,
        0);
  }

  // Resolve output diffuse parameter.
  mPSOutDiffuse = psMain->resolveOutputParameter(
      Ogre::RTShader::Parameter::SPS_COLOR, 0,
      Ogre::RTShader::Parameter::SPC_COLOR_DIFFUSE, Ogre::GCT_FLOAT4);

  // Get in/local specular parameter.
  mPSSpecualr = psMain->getParameterBySemantic(
      psMain->getInputParameters(), Ogre::RTShader::Parameter::SPS_COLOR,
      1);
  if (mPSSpecualr.get() == nullptr)
  {
    mPSSpecualr = psMain->getParameterBySemantic(
        psMain->getLocalParameters(), Ogre::RTShader::Parameter::SPS_COLOR,
        1);
  }

  // Resolve computed local shadow colour parameter.
  mPSLocalShadowFactor = psMain->resolveLocalParameter(
      Ogre::RTShader::Parameter::SPS_UNKNOWN, 0, "lShadowFactor",
      Ogre::GCT_FLOAT1);

  // Resolve computed local shadow colour parameter.
  mPSSplitPoints = psProgram->resolveParameter(Ogre::GCT_FLOAT4, -1,
      (Ogre::uint16)Ogre::GPV_GLOBAL, "pssm_split_points");

  // Get derived scene colour.
  mPSDerivedSceneColour = psProgram->resolveAutoParameterInt(
      Ogre::GpuProgramParameters::ACT_DERIVED_SCENE_COLOUR, 0);

  auto it = mShadowTextureParamsList.begin();
  int lightIndex = 0;

  while (it != mShadowTextureParamsList.end())
  {
    it->mWorldViewProjMatrix = vsProgram->resolveParameter(
        Ogre::GCT_MATRIX_4X4, -1,
        static_cast<Ogre::uint16>(Ogre::GPV_PER_OBJECT),
        "world_texture_view_proj");

    it->mVSOutLightPosition = vsMain->resolveOutputParameter(
        Ogre::RTShader::Parameter::SPS_TEXTURE_COORDINATES, -1,
        Ogre::RTShader::Parameter::Content(
          Ogre::RTShader::Parameter::SPC_POSITION_LIGHT_SPACE0 + lightIndex),
        Ogre::GCT_FLOAT4);

    it->mPSInLightPosition = psMain->resolveInputParameter(
        Ogre::RTShader::Parameter::SPS_TEXTURE_COORDINATES,
        it->mVSOutLightPosition->getIndex(),
        it->mVSOutLightPosition->getContent(),
        Ogre::GCT_FLOAT4);

    // Changed to enable hardware PCF
    // it->mTextureSampler = psProgram->resolveParameter(
    //     Ogre::GCT_SAMPLER2D, it->mTextureSamplerIndex,
    //     static_cast<Ogre::uint16>(Ogre::GPV_GLOBAL),
    //     "shadow_map");
    it->mTextureSampler = psProgram->resolveParameter(
        Ogre::GCT_SAMPLER2DSHADOW, it->mTextureSamplerIndex,
        static_cast<Ogre::uint16>(Ogre::GPV_GLOBAL), "shadow_map");

    it->mInvTextureSize = psProgram->resolveParameter(Ogre::GCT_FLOAT4, -1,
        static_cast<Ogre::uint16>(Ogre::GPV_GLOBAL), "inv_shadow_texture_size");

    if (!(it->mInvTextureSize.get()) || !(it->mTextureSampler.get()) ||
        !(it->mPSInLightPosition.get()) ||
        !(it->mVSOutLightPosition.get()) ||
        !(it->mWorldViewProjMatrix.get()))
    {
      OGRE_EXCEPT(Ogre::Exception::ERR_INTERNAL_ERROR,
        "Not all parameters could be constructed for the sub-render state.",
        "IntegratedPSSM3::resolveParameters");
    }

    ++lightIndex;
    ++it;
  }

  if (!(mVSInPos.get()) || !(mVSOutPos.get()) || !(mVSOutDepth.get()) ||
    !(mPSInDepth.get()) || !(mPSDiffuse.get()) || !(mPSOutDiffuse.get()) ||
    !(mPSSpecualr.get()) || !(mPSLocalShadowFactor.get()) ||
    !(mPSSplitPoints.get()) || !(mPSDerivedSceneColour.get()))
  {
    OGRE_EXCEPT(Ogre::Exception::ERR_INTERNAL_ERROR,
        "Not all parameters could be constructed for the sub-render state.",
        "IntegratedPSSM3::resolveParameters");
  }

  return true;
}

//////////////////////////////////////////////////
const Ogre::String &CustomPSSM3Factory::getType() const
{
  return CustomPSSM3::Type;
}

//////////////////////////////////////////////////
Ogre::RTShader::SubRenderState *CustomPSSM3Factory::createInstance(
    Ogre::ScriptCompiler *_compiler,
    Ogre::PropertyAbstractNode *_prop, Ogre::Pass * /*_pass*/,
    Ogre::RTShader::SGScriptTranslator *_translator)
{
  if (_prop->name == "integrated_pssm4")
  {
    if (_prop->values.size() != 4)
    {
       _compiler->addError(Ogre::ScriptCompiler::CE_INVALIDPARAMETERS,
          _prop->file, _prop->line);
    }
    else
    {
      CustomPSSM3::SplitPointList splitPointList;

      Ogre::AbstractNodeList::const_iterator it = _prop->values.begin();
      Ogre::AbstractNodeList::const_iterator itEnd = _prop->values.end();

      while (it != itEnd)
      {
        Ogre::Real curSplitValue;

        if (false == Ogre::RTShader::SGScriptTranslator::getReal(
            *it, &curSplitValue))
        {
          _compiler->addError(Ogre::ScriptCompiler::CE_INVALIDPARAMETERS,
              _prop->file,  _prop->line);
          break;
        }

        splitPointList.push_back(curSplitValue);

        ++it;
      }

      if (splitPointList.size() == 4)
      {
        Ogre::RTShader::SubRenderState *subRenderState =

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 7
            this->createInstance();
#else
            this->createOrRetrieveInstance(_translator);
#endif
        CustomPSSM3 *pssmSubRenderState =
            static_cast<CustomPSSM3 *>(subRenderState);

        pssmSubRenderState->setSplitPoints(splitPointList);

        return pssmSubRenderState;
      }
    }
  }

  return nullptr;
}

//////////////////////////////////////////////////
Ogre::RTShader::SubRenderState *CustomPSSM3Factory::createInstanceImpl()
{
  return OGRE_NEW CustomPSSM3;
}

//////////////////////////////////////////////////
CustomPSSMShadowCameraSetup::CustomPSSMShadowCameraSetup()
{
}

//////////////////////////////////////////////////
CustomPSSMShadowCameraSetup::~CustomPSSMShadowCameraSetup()
{
}

//////////////////////////////////////////////////
void CustomPSSMShadowCameraSetup::calculateShadowMappingMatrix(
    const Ogre::SceneManager &_sm, const Ogre::Camera &_cam,
    const Ogre::Light &_light, Ogre::Matrix4 *_outView,
    Ogre::Matrix4 *_outProj, Ogre::Camera *outCam) const
{
  // get the shadow frustum's far distance
  Ogre::Real shadowDist = _light.getShadowFarDistance();
  if (shadowDist <= 0)
  {
    // need a shadow distance, make one up
    shadowDist = _cam.getNearClipDistance() * 3000;
  }
  Ogre::Real shadowOffset = shadowDist * _sm.getShadowDirLightTextureOffset();


  if (_light.getType() == Ogre::Light::LT_DIRECTIONAL)
  {
    // generate view matrix if requested
    if (_outView != nullptr)
    {
      Ogre::Vector3 pos;
      if (_sm.getCameraRelativeRendering())
      {
        pos = Ogre::Vector3::ZERO;
      }
      else
      {
        pos = _cam.getDerivedPosition();
      }
      *_outView = this->buildViewMatrix(pos,
        _light.getDerivedDirection(),
        // Modified for z-up light frusta
        Ogre::Vector3::UNIT_Z);
    }

    // generate projection matrix if requested
    if (_outProj != nullptr)
    {
      *_outProj = Ogre::Matrix4::getScale(1, 1, -1);
    }

    // set up camera if requested
    if (outCam != nullptr)
    {
      outCam->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
      outCam->setDirection(_light.getDerivedDirection());
      outCam->setPosition(_cam.getDerivedPosition());
      outCam->setFOVy(Ogre::Degree(90));
      outCam->setNearClipDistance(shadowOffset);
    }
  }
  else if (_light.getType() == Ogre::Light::LT_POINT)
  {
    // target analogue to the default shadow textures
    // Calculate look at position
    // We want to look at a spot shadowOffset away from near plane
    // 0.5 is a little too close for angles
    Ogre::Vector3 target = _cam.getDerivedPosition() +
      (_cam.getDerivedDirection() * shadowOffset);
    Ogre::Vector3 lightDir = target - _light.getDerivedPosition();
    lightDir.normalise();

    // generate view matrix if requested
    if (_outView != nullptr)
    {
      *_outView = this->buildViewMatrix(_light.getDerivedPosition(),
        lightDir,
        _cam.getDerivedUp());
    }

    // generate projection matrix if requested
    if (_outProj != nullptr)
    {
      // set FOV to 120 degrees
      mTempFrustum->setFOVy(Ogre::Degree(120));

      mTempFrustum->setNearClipDistance(
          _light._deriveShadowNearClipDistance(&_cam));
      mTempFrustum->setFarClipDistance(
          _light._deriveShadowFarClipDistance(&_cam));

      *_outProj = mTempFrustum->getProjectionMatrix();
    }

    // set up camera if requested
    if (outCam != nullptr)
    {
      outCam->setProjectionType(Ogre::PT_PERSPECTIVE);
      outCam->setDirection(lightDir);
      outCam->setPosition(_light.getDerivedPosition());
      outCam->setFOVy(Ogre::Degree(120));
      outCam->setNearClipDistance(_light._deriveShadowNearClipDistance(&_cam));
      outCam->setFarClipDistance(_light._deriveShadowFarClipDistance(&_cam));
    }
  }
  else if (_light.getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    // generate view matrix if requested
    if (_outView != nullptr)
    {
      *_outView = this->buildViewMatrix(_light.getDerivedPosition(),
        _light.getDerivedDirection(),
        _cam.getDerivedUp());
    }

    // generate projection matrix if requested
    if (_outProj != nullptr)
    {
      // set FOV slightly larger than spotlight range
      mTempFrustum->setFOVy(Ogre::Math::Clamp<Ogre::Radian>(
          _light.getSpotlightOuterAngle() * 1.2f, Ogre::Radian(0),
          Ogre::Radian(Ogre::Math::PI/2.0f)));

      mTempFrustum->setNearClipDistance(
          _light._deriveShadowNearClipDistance(&_cam));
      mTempFrustum->setFarClipDistance(
          _light._deriveShadowFarClipDistance(&_cam));

      *_outProj = mTempFrustum->getProjectionMatrix();
    }

    // set up camera if requested
    if (outCam != nullptr)
    {
      outCam->setProjectionType(Ogre::PT_PERSPECTIVE);
      outCam->setDirection(_light.getDerivedDirection());
      outCam->setPosition(_light.getDerivedPosition());
      outCam->setFOVy(Ogre::Math::Clamp<Ogre::Radian>(
          _light.getSpotlightOuterAngle() * 1.2f, Ogre::Radian(0),
          Ogre::Radian(Ogre::Math::PI/2.0f)));
      outCam->setNearClipDistance(_light._deriveShadowNearClipDistance(&_cam));
      outCam->setFarClipDistance(_light._deriveShadowFarClipDistance(&_cam));
    }
  }
}

//////////////////////////////////////////////////
Ogre::Matrix4 CustomPSSMShadowCameraSetup::buildViewMatrix(
    const Ogre::Vector3 &_pos, const Ogre::Vector3 &_dir,
    const Ogre::Vector3 &_up) const
{
  Ogre::Vector3 xN = _dir.crossProduct(_up);
  xN.normalise();
  Ogre::Vector3 upN = xN.crossProduct(_dir);
  upN.normalise();

  // Modified for z-up light frusta
  Ogre::Matrix4 m(xN.x,   xN.y,   xN.z,   -xN.dotProduct(_pos),
    _dir.x,   _dir.y, _dir.z, -_dir.dotProduct(_pos),
    upN.x,    upN.y,    upN.z,    -upN.dotProduct(_pos),
    0.0,      0.0,    0.0,    1.0);

  return m;
}

//////////////////////////////////////////////////
void CustomPSSMShadowCameraSetup::getZUpFocusedShadowCamera(
    const Ogre::SceneManager *_sm, const Ogre::Camera *_cam,
    const Ogre::Viewport * /*_vp*/, const Ogre::Light *_light,
    Ogre::Camera *_texCam, size_t /*_iteration*/) const
{
  // check availability - viewport not needed
  OgreAssert(_sm != nullptr, "SceneManager is NULL");
  OgreAssert(_cam != nullptr, "Camera (viewer) is NULL");
  OgreAssert(_light != nullptr, "Light is nullptr");
  OgreAssert(_texCam != nullptr, "Camera (texture) is NULL");
  mLightFrustumCameraCalculated = false;

  _texCam->setNearClipDistance(_light->_deriveShadowNearClipDistance(_cam));
  _texCam->setFarClipDistance(_light->_deriveShadowFarClipDistance(_cam));

  // calculate standard shadow mapping matrix
  Ogre::Matrix4 LView, LProj;
  this->calculateShadowMappingMatrix(*_sm, *_cam, *_light, &LView, &LProj,
      nullptr);

  // build scene bounding box
  const Ogre::VisibleObjectsBoundsInfo& visInfo =
      _sm->getVisibleObjectsBoundsInfo(_texCam);
  Ogre::AxisAlignedBox sceneBB = visInfo.aabb;
  Ogre::AxisAlignedBox receiverAABB =
      _sm->getVisibleObjectsBoundsInfo(_cam).receiverAabb;
  sceneBB.merge(receiverAABB);
  sceneBB.merge(_cam->getDerivedPosition());

  // in case the sceneBB is empty (e.g. nothing visible to the cam) simply
  // return the standard shadow mapping matrix
  if (sceneBB.isNull())
  {
    _texCam->setCustomViewMatrix(true, LView);
    _texCam->setCustomProjectionMatrix(true, LProj);
    return;
  }

  // calculate the intersection body B
  mPointListBodyB.reset();
  this->calculateB(*_sm, *_cam, *_light, sceneBB, receiverAABB,
      &mPointListBodyB);

  // in case the bodyB is empty (e.g. nothing visible to the light or the cam)
  // simply return the standard shadow mapping matrix
  if (mPointListBodyB.getPointCount() == 0)
  {
    _texCam->setCustomViewMatrix(true, LView);
    _texCam->setCustomProjectionMatrix(true, LProj);
    return;
  }

  // transform to light space: y -> -z, z -> y
  LProj = msNormalToLightSpace * LProj;

  // calculate LVS so it does not need to be calculated twice
  // calculate the body L \cap V \cap S to make sure all returned points are in
  // front of the camera
  // mPointListBodyLVS.reset();
  // calculateLVS(*_sm, *_cam, *_light, sceneBB, &mPointListBodyLVS);

  // fetch the viewing direction
  // const Vector3 viewDir = getLSProjViewDir(LProj * LView, *_cam,
  // mPointListBodyLVS);

  // The light space will be rotated in such a way, that the projected light
  // view always points upwards, so the up-vector is the y-axis (we already
  // prepared the light space for this usage).The transformation matrix is set
  // up with the following parameters:
  // - position is the origin
  // - the view direction is the calculated viewDir
  // - the up vector is the y-axis
  // Commented out for z-up light frusta
  // LProj = buildViewMatrix(Vector3::ZERO, viewDir, Vector3::UNIT_Y) * LProj;

  // map bodyB to unit cube
  LProj = this->transformToUnitCube(LProj * LView, mPointListBodyB) * LProj;

  // transform from light space to normal space: y -> z, z -> -y
  // Commented out for z-up light frusta
  // LProj = msLightSpaceToNormal * LProj;

  // set the two custom matrices
  _texCam->setCustomViewMatrix(true, LView);
  _texCam->setCustomProjectionMatrix(true, LProj);
}

//////////////////////////////////////////////////
void CustomPSSMShadowCameraSetup::getShadowCamera(const Ogre::SceneManager *_sm,
    const Ogre::Camera *_cam, const Ogre::Viewport *_vp,
    const Ogre::Light *_light, Ogre::Camera *_texCam, size_t _iteration) const
{
  // apply the right clip distance.
  Ogre::Real nearDist = mSplitPoints[_iteration];
  Ogre::Real farDist = mSplitPoints[_iteration + 1];

  // Add a padding factor to internal distances so that the connecting
  // split point will not have bad artifacts.
  if (_iteration > 0)
  {
    nearDist -= mSplitPadding;
    nearDist = std::max(nearDist, mSplitPoints[0]);
  }
  if (_iteration < mSplitCount - 1)
  {
    farDist += mSplitPadding;
  }

  mCurrentIteration = _iteration;

  // Ouch, I know this is hacky, but it's the easiest way to re-use LiSPSM /
  // Focused functionality right now without major changes
  Ogre::Camera *cam = const_cast<Ogre::Camera *>(_cam);
  Ogre::Real oldNear = _cam->getNearClipDistance();
  Ogre::Real oldFar = _cam->getFarClipDistance();
  cam->setNearClipDistance(nearDist);
  cam->setFarClipDistance(farDist);

  // Replaced LiSPSMShadowCameraSetup::getShadowCamera() with
  // FocusedShadowCameraSetup::getShadowCamera(). This is the same solution
  // they reached for Ogre 2.1 after realizing how buggy LiSPSM is.
  // However, unlike Ogre 2.1, we are using a modified method that makes a
  // shadow camera frustum aligned with the z-axis because Gazebo is z-up.
  this->getZUpFocusedShadowCamera(_sm, _cam, _vp, _light, _texCam, _iteration);

  // restore near/far
  cam->setNearClipDistance(oldNear);
  cam->setFarClipDistance(oldFar);
}

//////////////////////////////////////////////////
CustomGLSLProgramWriter::CustomGLSLProgramWriter()
  : Ogre::RTShader::GLSLProgramWriter()
{
  mGpuConstTypeMap[Ogre::GCT_SAMPLER2DSHADOW] = "sampler2DShadow";
}

//////////////////////////////////////////////////
CustomGLSLProgramWriterFactory::CustomGLSLProgramWriterFactory()
{
}

//////////////////////////////////////////////////
const Ogre::String &CustomGLSLProgramWriterFactory::getTargetLanguage() const
{
  static const Ogre::String targetLanguageGLSL("glsl");
  return targetLanguageGLSL;
}

//////////////////////////////////////////////////
Ogre::RTShader::ProgramWriter *CustomGLSLProgramWriterFactory::create()
{
  return OGRE_NEW CustomGLSLProgramWriter();
}
