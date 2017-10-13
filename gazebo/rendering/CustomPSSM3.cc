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

#include "gazebo/rendering/CustomPSSM3.hh"

#include <OgreShaderProgram.h>
#include <OgreShaderProgramSet.h>

using namespace gazebo;
using namespace rendering;

Ogre::String Ogre::RTShader::IntegratedPSSM3::Type = "CustomPSSM3";

//////////////////////////////////////////////////
const Ogre::String &CustomPSSM3::getType() const
{
  return Type;
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
    //     GCT_SAMPLER2D, it->mTextureSamplerIndex, (uint16)GPV_GLOBAL,
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
        "IntegratedPSSM3::resolveParameters" );
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
        "IntegratedPSSM3::resolveParameters" );
  }

  return true;
}

//////////////////////////////////////////////////
const Ogre::String &CustomPSSM3Factory::getType() const
{
  return Ogre::RTShader::IntegratedPSSM3::Type;
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
            this->createOrRetrieveInstance(_translator);
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
