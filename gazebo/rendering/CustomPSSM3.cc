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
#include "CustomPSSM3.hh"
#include <OgreShaderProgram.h>
#include <OgreShaderProgramSet.h>


namespace Ogre {
namespace RTShader {

String IntegratedPSSM3::Type = "CustomPSSM3";

//-----------------------------------------------------------------------
const String& CustomPSSM3::getType() const
{
  return Type;
}

//-----------------------------------------------------------------------
bool CustomPSSM3::resolveParameters(ProgramSet* programSet)
{
	Program* vsProgram = programSet->getCpuVertexProgram();
	Program* psProgram = programSet->getCpuFragmentProgram();
	Function* vsMain = vsProgram->getEntryPointFunction();
	Function* psMain = psProgram->getEntryPointFunction();

	// Get input position parameter.
	mVSInPos = vsMain->getParameterBySemantic(vsMain->getInputParameters(), Parameter::SPS_POSITION, 0);

	// Get output position parameter.
	mVSOutPos = vsMain->getParameterBySemantic(vsMain->getOutputParameters(), Parameter::SPS_POSITION, 0);

	// Resolve vertex shader output depth.
	mVSOutDepth = vsMain->resolveOutputParameter(Parameter::SPS_TEXTURE_COORDINATES, -1,
		Parameter::SPC_DEPTH_VIEW_SPACE,
		GCT_FLOAT1);

	// Resolve input depth parameter.
	mPSInDepth = psMain->resolveInputParameter(Parameter::SPS_TEXTURE_COORDINATES, mVSOutDepth->getIndex(),
		mVSOutDepth->getContent(),
		GCT_FLOAT1);

	// Get in/local diffuse parameter.
	mPSDiffuse = psMain->getParameterBySemantic(psMain->getInputParameters(), Parameter::SPS_COLOR, 0);
	if (mPSDiffuse.get() == NULL)
	{
		mPSDiffuse = psMain->getParameterBySemantic(psMain->getLocalParameters(), Parameter::SPS_COLOR, 0);
	}

	// Resolve output diffuse parameter.
	mPSOutDiffuse = psMain->resolveOutputParameter(Parameter::SPS_COLOR, 0, Parameter::SPC_COLOR_DIFFUSE, GCT_FLOAT4);

	// Get in/local specular parameter.
	mPSSpecualr = psMain->getParameterBySemantic(psMain->getInputParameters(), Parameter::SPS_COLOR, 1);
	if (mPSSpecualr.get() == NULL)
	{
		mPSSpecualr = psMain->getParameterBySemantic(psMain->getLocalParameters(), Parameter::SPS_COLOR, 1);
	}

	// Resolve computed local shadow colour parameter.
	mPSLocalShadowFactor = psMain->resolveLocalParameter(Parameter::SPS_UNKNOWN, 0, "lShadowFactor", GCT_FLOAT1);

	// Resolve computed local shadow colour parameter.
	mPSSplitPoints = psProgram->resolveParameter(GCT_FLOAT4, -1, (uint16)GPV_GLOBAL, "pssm_split_points");

	// Get derived scene colour.
	mPSDerivedSceneColour = psProgram->resolveAutoParameterInt(GpuProgramParameters::ACT_DERIVED_SCENE_COLOUR, 0);

	ShadowTextureParamsIterator it = mShadowTextureParamsList.begin();
	int lightIndex = 0;

	while(it != mShadowTextureParamsList.end())
	{
		it->mWorldViewProjMatrix = vsProgram->resolveParameter(GCT_MATRIX_4X4, -1, (uint16)GPV_PER_OBJECT, "world_texture_view_proj");

		it->mVSOutLightPosition = vsMain->resolveOutputParameter(Parameter::SPS_TEXTURE_COORDINATES, -1,
			Parameter::Content(Parameter::SPC_POSITION_LIGHT_SPACE0 + lightIndex),
			GCT_FLOAT4);

		it->mPSInLightPosition = psMain->resolveInputParameter(Parameter::SPS_TEXTURE_COORDINATES,
			it->mVSOutLightPosition->getIndex(),
			it->mVSOutLightPosition->getContent(),
			GCT_FLOAT4);

    // Changed to enable hardware PCF
    // it->mTextureSampler = psProgram->resolveParameter(GCT_SAMPLER2D, it->mTextureSamplerIndex, (uint16)GPV_GLOBAL, "shadow_map");
    it->mTextureSampler = psProgram->resolveParameter(GCT_SAMPLER2DSHADOW, it->mTextureSamplerIndex, (uint16)GPV_GLOBAL, "shadow_map");

		it->mInvTextureSize = psProgram->resolveParameter(GCT_FLOAT4, -1, (uint16)GPV_GLOBAL, "inv_shadow_texture_size");

		if (!(it->mInvTextureSize.get()) || !(it->mTextureSampler.get()) || !(it->mPSInLightPosition.get()) ||
			!(it->mVSOutLightPosition.get()) || !(it->mWorldViewProjMatrix.get()))
		{
			OGRE_EXCEPT( Exception::ERR_INTERNAL_ERROR,
				"Not all parameters could be constructed for the sub-render state.",
				"IntegratedPSSM3::resolveParameters" );
		}

		++lightIndex;
		++it;
	}

	if (!(mVSInPos.get()) || !(mVSOutPos.get()) || !(mVSOutDepth.get()) || !(mPSInDepth.get()) || !(mPSDiffuse.get()) || !(mPSOutDiffuse.get()) ||
		!(mPSSpecualr.get()) || !(mPSLocalShadowFactor.get()) || !(mPSSplitPoints.get()) || !(mPSDerivedSceneColour.get()))
	{
		OGRE_EXCEPT( Exception::ERR_INTERNAL_ERROR,
				"Not all parameters could be constructed for the sub-render state.",
				"IntegratedPSSM3::resolveParameters" );
  }

	return true;
}


//-----------------------------------------------------------------------
const String& CustomPSSM3Factory::getType() const
{
  return IntegratedPSSM3::Type;
}

//-----------------------------------------------------------------------
SubRenderState* CustomPSSM3Factory::createInstance(ScriptCompiler* compiler,
  PropertyAbstractNode* prop, Pass* pass, SGScriptTranslator* translator)
{
  if (prop->name == "integrated_pssm4")
  {
    if (prop->values.size() != 4)
    {
       compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line);
    }
    else
    {
      CustomPSSM3::SplitPointList splitPointList;

      AbstractNodeList::const_iterator it = prop->values.begin();
      AbstractNodeList::const_iterator itEnd = prop->values.end();

      while(it != itEnd)
      {
        Real curSplitValue;

        if (false == SGScriptTranslator::getReal(*it, &curSplitValue))
        {
          compiler->addError(ScriptCompiler::CE_INVALIDPARAMETERS, prop->file, prop->line);
          break;
        }

        splitPointList.push_back(curSplitValue);

        ++it;
      }

      if (splitPointList.size() == 4)
      {
        SubRenderState* subRenderState = createOrRetrieveInstance(translator);
        CustomPSSM3* pssmSubRenderState = static_cast<CustomPSSM3*>(subRenderState);

        pssmSubRenderState->setSplitPoints(splitPointList);

        return pssmSubRenderState;
      }
    }
  }

  return NULL;
}

//-----------------------------------------------------------------------
SubRenderState* CustomPSSM3Factory::createInstanceImpl()
{
  return OGRE_NEW CustomPSSM3;
}


}
}

//#endif
