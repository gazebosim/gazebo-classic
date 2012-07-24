/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include <OgreMaterialManager.h>
#include <OgreGpuProgramManager.h>
#include <OgreStringConverter.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreTechnique.h>

#include "gazebo/rendering/deferred_shading/GBufferMaterialGenerator.hh"

// Use this directive to control whether you are writing
// projective (regular) or linear depth.
#define WRITE_LINEAR_DEPTH

using namespace gazebo;
using namespace rendering;

// This is the concrete implementation of the material generator.
class GBufferMaterialGeneratorImpl : public MaterialGenerator::Impl
{
  public: GBufferMaterialGeneratorImpl(const Ogre::String &_baseName,
              GBufferMaterialGenerator::GBufferType _type)
          : baseName(_baseName), type(_type) {}

  public: void SetType(GBufferMaterialGenerator::GBufferType _type)
          {this->type = _type;};

  protected: virtual Ogre::GpuProgramPtr GenerateVertexShader(
                 MaterialGenerator::Perm permutation);
  protected: virtual Ogre::GpuProgramPtr GenerateFragmentShader(
                 MaterialGenerator::Perm permutation);
  protected: virtual Ogre::MaterialPtr GenerateTemplateMaterial(
                 MaterialGenerator::Perm permutation);

  protected: Ogre::String baseName;
  private: GBufferMaterialGenerator::GBufferType type;
};

/////////////////////////////////////////////////
GBufferMaterialGenerator::GBufferMaterialGenerator(GBufferType _type)
  : MaterialGenerator()
{
  vsMask = VS_MASK;
  fsMask = FS_MASK;
  matMask = MAT_MASK;

  if (_type == GBT_FAT)
  {
    materialBaseName = "DeferredShading";
    this->schemeName = "DSGBuffer";
  }
  else if (_type == GBT_NORMAL_AND_DEPTH)
  {
    materialBaseName = "DeferredLighting";
    this->schemeName = "DLGBuffer";
  }
  else if (_type == GBT_DSF)
  {
    materialBaseName = "InferredLighting";
    this->schemeName = "ILGBuffer";
  }

  materialBaseName += "/GBuffer/";
  this->impl = new GBufferMaterialGeneratorImpl(materialBaseName, _type);
}

/////////////////////////////////////////////////
Ogre::GpuProgramPtr GBufferMaterialGeneratorImpl::GenerateVertexShader(
    MaterialGenerator::Perm _permutation)
{
  Ogre::StringStream ss;
  uint32_t numTexCoords = (_permutation &
      GBufferMaterialGenerator::GBP_TEXCOORD_MASK) >> 8;

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
    ss << "attribute vec3 tangentMap;\n";

  ss << "uniform mat4 worldViewProj;\n";
  ss << "uniform mat4 worldView;\n";

#ifdef WRITE_LINEAR_DEPTH
  ss << "varying vec3 viewPos;\n";
#else
  ss << "varying float depth;\n";
#endif
  ss << "varying vec3 normal;\n";

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
  {
    ss << "out vec3 tangent;\n";
    ss << "out vec3 biNormal;\n";
  }

  ss << "void main()\n{\n";
  ss << "  gl_Position = worldViewProj * gl_Vertex;\n";
  ss << "  normal = (worldView * vec4(gl_Normal, 0.0)).xyz;\n";

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
  {
    // THis is probably wrong because I copied it incorrectly
    ss << " tangent = (worldView * vec4(tangentMap, 0)).xyz;\n";
    ss << " biNormal = cross(normal, tangent);\n";
  }

#ifdef WRITE_LINEAR_DEPTH
  ss << "  viewPos = (worldView * gl_Vertex).xyz;\n";
#else
  ss << "  depth = gl_Position.w;\n";
#endif

  for (uint32_t i = 0; i < numTexCoords; ++i)
  {
    ss << "  gl_TexCoord[" << 0 << "] = gl_MultiTexCoord" << 0 << ";\n";
  }
  ss << "}\n";

  Ogre::String programSource = ss.str();
  Ogre::String programName = this->baseName + "VP_" +
                             Ogre::StringConverter::toString(_permutation);

#if OGRE_DEBUG_MODE
  Ogre::LogManager::getSingleton().getDefaultLog()->logMessage(programSource);
#endif

  // Create shader object
  Ogre::HighLevelGpuProgramPtr ptrProgram =
    Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
        programName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        "glsl", Ogre::GPT_VERTEX_PROGRAM);

  ptrProgram->setSource(programSource);

  const Ogre::GpuProgramParametersSharedPtr &params =
    ptrProgram->getDefaultParameters();
  params->setNamedAutoConstant("worldViewProj",
      Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
  params->setNamedAutoConstant("worldView",
      Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX);
  ptrProgram->load();

  return Ogre::GpuProgramPtr(ptrProgram);
}
/////////////////////////////////////////////////
Ogre::GpuProgramPtr GBufferMaterialGeneratorImpl::GenerateFragmentShader(
    MaterialGenerator::Perm _permutation)
{
  Ogre::StringStream ss;

  uint32_t numTexCoords =
    (_permutation & GBufferMaterialGenerator::GBP_TEXCOORD_MASK) >> 8;
  uint32_t numTextures =
    _permutation & GBufferMaterialGenerator::GBP_TEXTURE_MASK;

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
    ss << "uniform sampler2D normalMap;\n";

  if (this->type == GBufferMaterialGenerator::GBT_FAT)
  {
    for (uint32_t i = 0; i < numTextures; ++i)
      ss << "uniform sampler2D tex" << i << ";\n";
  }

  if (numTextures == 0 ||
      _permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
  {
    if (this->type == GBufferMaterialGenerator::GBT_FAT)
      ss << "uniform vec4 diffuseColor;\n";
  }

#ifdef WRITE_LINEAR_DEPTH
  ss << "uniform float farDistance;\n";
#endif

  if (this->type == GBufferMaterialGenerator::GBT_FAT)
    ss << "uniform float specularity;\n";
  else if (this->type == GBufferMaterialGenerator::GBT_DSF)
    ss << "uniform vec4 objectId;\n";

#ifdef WRITE_LINEAR_DEPTH
  ss << "varying vec3 viewPos;\n";
#else
  ss << "varying float depth;\n";
#endif
  ss << "varying vec3 normal;\n";

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
  {
    ss << "varying vec3 tangent;\n";
    ss << "varying vec3 biNormal;\n";
  }

  ss << "void main()\n{\n";

  if (this->type == GBufferMaterialGenerator::GBT_FAT)
  {
    if (numTexCoords > 0 && numTextures > 0)
    {
      ss << "  gl_FragData[1].xyz = texture2D(tex0, gl_TexCoord[0].st);\n";
      if (_permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
        ss << "  gl_FragData[1].xyz *= diffuseColor.xyz;\n";
    }
    else
    {
      ss << "  gl_FragData[1].xyz = diffuseColor.xyz;\n";
    }
    ss << "  gl_FragData[1].w = specularity;\n";
  }


  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
  {
    ss << "  vec3 texNormal = "
       << "texture2D(normalMap, gl_TexCoord[0].st) - 0.5) * 2.0;\n";
    ss << "  mat3 normalRotation = mat3(tangent, biNormal, normal);\n";
    ss << "  gl_FragData[0].xyz = normalize(texNormal * normalRotation);\n";
  }
  else
  {
    ss << "  gl_FragData[0].xyz = normalize(normal);\n";
  }

#ifdef WRITE_LINEAR_DEPTH
  ss << "  gl_FragData[0].w = length(viewPos) / farDistance;\n";
#else
  ss << "  gl_FragData[0].w = depth;\n";
#endif

  if (this->type == GBufferMaterialGenerator::GBT_DSF)
  {
    ss << "  vec3 norm = normalize(normal);\n";
    ss << "  gl_FragData[1].x = length(viewPos) / farDistance;\n";
    ss << "  gl_FragData[1].y = objectId.r;\n";
    ss << "  gl_FragData[1].z = normal.x;\n";
    ss << "  gl_FragData[1].w = normal.y;\n";
  }

  ss << "}\n";

  Ogre::String programSource = ss.str();
  Ogre::String programName = this->baseName + "FP_" +
                             Ogre::StringConverter::toString(_permutation);

#if OGRE_DEBUG_MODE
  Ogre::LogManager::getSingleton().getDefaultLog()->logMessage(programSource);
#endif

  // Create shader object
  Ogre::HighLevelGpuProgramPtr ptrProgram =
    Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
        programName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        "glsl", Ogre::GPT_FRAGMENT_PROGRAM);

  ptrProgram->setSource(programSource);

  const Ogre::GpuProgramParametersSharedPtr &params =
    ptrProgram->getDefaultParameters();

  if (this->type == GBufferMaterialGenerator::GBT_FAT)
  {
    params->setNamedAutoConstant("specularity",
        Ogre::GpuProgramParameters::ACT_SURFACE_SHININESS);

    if (numTextures == 0 ||
        _permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
    {
      params->setNamedAutoConstant("diffuseColor",
          Ogre::GpuProgramParameters::ACT_SURFACE_DIFFUSE_COLOUR);
    }
  }

  if (this->type == GBufferMaterialGenerator::GBT_DSF)
  {
    params->setNamedAutoConstant("objectId",
        Ogre::GpuProgramParameters::ACT_CUSTOM, 0);
  }

#ifdef WRITE_LINEAR_DEPTH
  // TODO : Should this be the distance to the far corner,
  //        not the far clip distance?
  params->setNamedAutoConstant("farDistance",
      Ogre::GpuProgramParameters::ACT_FAR_CLIP_DISTANCE);
#endif

  ptrProgram->load();
  return Ogre::GpuProgramPtr(ptrProgram);
}

/////////////////////////////////////////////////
Ogre::MaterialPtr GBufferMaterialGeneratorImpl::GenerateTemplateMaterial(
    MaterialGenerator::Perm _permutation)
{
  Ogre::String matName = this->baseName + "Mat_" +
                         Ogre::StringConverter::toString(_permutation);

  Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create
    (matName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  Ogre::Pass *pass = matPtr->getTechnique(0)->getPass(0);
  pass->setName(this->baseName + "Pass_" +
                Ogre::StringConverter::toString(_permutation));
  pass->setLightingEnabled(false);

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
    pass->createTextureUnitState();

  uint32_t numTextures = _permutation & GBufferMaterialGenerator::GBP_TEXTURE_MASK;

  for (uint32_t i = 0; i < numTextures; ++i)
    pass->createTextureUnitState();

  return matPtr;
}

///////////////////////////////////////////////////////////////////////////////
//CG Code take from OGRE sample. DELETE ME
//
/*Ogre::GpuProgramPtr GBufferMaterialGeneratorImpl::GenerateVertexShader(
    MaterialGenerator::Perm permutation)
{
	Ogre::StringStream ss;
	
	ss << "void ToGBufferVP(" << std::endl;
	ss << "	float4 iPosition : POSITION," << std::endl;
	ss << "	half3 iNormal   : NORMAL," << std::endl;

	Ogre::uint32 numTexCoords = (permutation & GBufferMaterialGenerator::GBP_TEXCOORD_MASK) >> 8;
	for (Ogre::uint32 i=0; i<numTexCoords; i++) 
	{
		ss << "	half2 iUV" << i << " : TEXCOORD" << i << ',' << std::endl;
	}

	if (permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
	{
		ss << "	half3 iTangent : TANGENT0," << std::endl;
	}

	//TODO : Skinning inputs
	ss << std::endl;
	


	ss << "	out float4 oPosition : POSITION," << std::endl;
#ifdef WRITE_LINEAR_DEPTH
    ss << "	out float3 oViewPos : TEXCOORD0," << std::endl;
#else
	ss << "	out float oDepth : TEXCOORD0," << std::endl;
#endif
	int texCoordNum=2;
	ss << "	out half3 oNormal : TEXCOORD1," << std::endl;
	

	if ((permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP) ) 
	{
		ss << "	out half3 oTangent : TEXCOORD" << texCoordNum++ << ',' << std::endl;
		ss << "	out half3 oBiNormal : TEXCOORD" << texCoordNum++ << ',' << std::endl;
	}
	for (Ogre::uint32 i=0; i<numTexCoords; i++) 
	{
		ss << "	out half2 oUV" << i << " : TEXCOORD" << texCoordNum++ << ',' << std::endl;
	}

	ss << std::endl;

	ss << "	uniform float4x4 cWorldViewProj," << std::endl;
	ss << "	uniform float4x4 cWorldView" << std::endl;

	ss << "	)" << std::endl;
	
	
	ss << "{" << std::endl;
	ss << "	oPosition = mul(cWorldViewProj, iPosition);" << std::endl;
	ss << "	oNormal = mul(cWorldView, float4(iNormal,0)).xyz;" << std::endl;
	if (permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP )
	{
		ss << "	oTangent = mul(cWorldView, float4(iTangent,0)).xyz;" << std::endl;
		ss << "	oBiNormal = cross(oNormal, oTangent);" << std::endl;
	}

#ifdef WRITE_LINEAR_DEPTH
    ss << "	oViewPos = mul(cWorldView, iPosition).xyz;" << std::endl;
#else
	ss << "	oDepth = oPosition.w;" << std::endl;
#endif
	for (Ogre::uint32 i=0; i<numTexCoords; i++) {
		ss << "	oUV" << i << " = iUV" << i << ';' << std::endl;
	}

	ss << "}" << std::endl;
	
	Ogre::String programSource = ss.str();

	Ogre::String programName = this->baseName + "VP_" + Ogre::StringConverter::toString(permutation);
#if OGRE_DEBUG_MODE
	Ogre::LogManager::getSingleton().getDefaultLog()->logMessage(programSource);
#endif

	// Create shader object
	Ogre::HighLevelGpuProgramPtr ptrProgram = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
		programName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		"cg", Ogre::GPT_VERTEX_PROGRAM);
	ptrProgram->setSource(programSource);
	ptrProgram->setParameter("entry_point","ToGBufferVP");
	ptrProgram->setParameter("profiles","vs_1_1 arbvp1");

	const Ogre::GpuProgramParametersSharedPtr& params = ptrProgram->getDefaultParameters();
	params->setNamedAutoConstant("cWorldViewProj", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
	params->setNamedAutoConstant("cWorldView", Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX);
	ptrProgram->load();

	return Ogre::GpuProgramPtr(ptrProgram);
}*/



/*Ogre::GpuProgramPtr GBufferMaterialGeneratorImpl::GenerateFragmentShader(
    MaterialGenerator::Perm permutation)
{
  Ogre::StringStream ss;

  ss << "void ToGBufferFP(" << std::endl;
#ifdef WRITE_LINEAR_DEPTH
  ss << "	float3 iViewPos : TEXCOORD0," << std::endl;
#else
  ss << "	float1 iDepth : TEXCOORD0," << std::endl;
#endif
  ss << "	float3 iNormal   : TEXCOORD1" ;

  int texCoordNum = 2;
  if (permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP ) 
  {
    ss << ',' << std::endl<< "	float3 iTangent : TEXCOORD" << texCoordNum++ ;
    ss << ',' << std::endl << "	half3 iBiNormal : TEXCOORD" << texCoordNum++ ;
  }

  Ogre::uint32 numTexCoords = (permutation & GBufferMaterialGenerator::GBP_TEXCOORD_MASK) >> 8;
  for (Ogre::uint32 i=0; i<numTexCoords; i++) 
  {
    ss << ',' << std::endl<< "	half2 iUV" << i << " : TEXCOORD" << texCoordNum++ ;
  }

  ss <<','<< std::endl<< "	out half4 oColor0 : COLOR0" ;
  //If we're not using deferred lighting, the GBuffer needs a second channel
  if(this->type != GBufferMaterialGenerator::GBT_NORMAL_AND_DEPTH){
    ss <<','<< std::endl<< "	out half4 oColor1 : COLOR1" ;
  }



  int samplerNum = 0;
  if (permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
  {
    ss <<","<<std::endl<< "	uniform sampler sNormalMap : register(s" << samplerNum++ << ")";
  }

  Ogre::uint32 numTextures = permutation & GBufferMaterialGenerator::GBP_TEXTURE_MASK;

  if(this->type == GBufferMaterialGenerator::GBT_FAT){
    for (Ogre::uint32 i=0; i<numTextures; i++) {
      ss << ","<<std::endl<<"uniform sampler sTex" << i << " : register(s" << samplerNum++ << ")";
    }
  }
  if (numTextures == 0 || permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
  {
    if(this->type == GBufferMaterialGenerator::GBT_FAT)
      ss<<","<< std::endl << "	uniform half4 cDiffuseColour" ;
  }

#ifdef WRITE_LINEAR_DEPTH
  ss <<","<<std::endl<< "	uniform float cFarDistance";
#endif
  ss<< std::endl;
  //Only use specularity for deferred shading
  if(this->type == GBufferMaterialGenerator::GBT_FAT){
    ss <<","<<std::endl<< "	uniform half cSpecularity" << std::endl;
  }
  //This is an unique ID for each instance of an object. Used in Inferred Lighting
  if(this->type == GBufferMaterialGenerator::GBT_DSF){
    ss <<","<<std::endl<< "	uniform half4 cObjectId" << std::endl;
  }
  ss << "	)" << std::endl;


  ss << "{" << std::endl;
  //Only get Albedo for deferred shading
  if( this->type == GBufferMaterialGenerator::GBT_FAT){
    if (numTexCoords > 0 && numTextures > 0 ) 
    {
      ss << "	oColor1.rgb = tex2D(sTex0, iUV0);" << std::endl;
      if (permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
      {
        ss << "	oColor1.rgb *= cDiffuseColour.rgb;" << std::endl;
      }
    }
    else
    {
      ss << "	oColor1.rgb = cDiffuseColour.rgb;" << std::endl;
    }
    ss << "	oColor1.a = cSpecularity;" << std::endl;
  }


  if (permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP) 
  {
    ss << "	half3 texNormal = (tex2D(sNormalMap, iUV0)-0.5)*2;" << std::endl;
    ss << "	float3x3 normalRotation = float3x3(iTangent, iBiNormal, iNormal);" << std::endl;
    ss << "	oColor0.rgb = normalize(mul(texNormal, normalRotation));" << std::endl;
  } else 
  {
    ss << "	oColor0.rgb = normalize(iNormal);" << std::endl;
  }
#ifdef WRITE_LINEAR_DEPTH
  ss << "	oColor0.a = length(iViewPos) / cFarDistance;" << std::endl;
#else
  ss << "	oColor1.a = iDepth;" << std::endl;
#endif
  if(this->type == GBufferMaterialGenerator::GBT_DSF){
    ss << "	iNormal = normalize(iNormal);oColor1.r = length(iViewPos) / cFarDistance;" << std::endl;
    ss << "	oColor1.g = cObjectId.r;" << std::endl;
    ss << "	oColor1.b = iNormal.x;" << std::endl;
    ss << "	oColor1.a = iNormal.y;" << std::endl;
  }
  ss << "}" << std::endl;

  Ogre::String programSource = ss.str();
  Ogre::String programName = this->baseName + "FP_" + Ogre::StringConverter::toString(permutation);

#if OGRE_DEBUG_MODE
  Ogre::LogManager::getSingleton().getDefaultLog()->logMessage(programSource);
#endif

  // Create shader object
  Ogre::HighLevelGpuProgramPtr ptrProgram = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
      programName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      "cg", Ogre::GPT_FRAGMENT_PROGRAM);
  ptrProgram->setSource(programSource);
  ptrProgram->setParameter("entry_point","ToGBufferFP");
  ptrProgram->setParameter("profiles","ps_3_0 arbfp1");

  const Ogre::GpuProgramParametersSharedPtr& params = ptrProgram->getDefaultParameters();
  if( this->type == GBufferMaterialGenerator::GBT_FAT){
    params->setNamedAutoConstant("cSpecularity", Ogre::GpuProgramParameters::ACT_SURFACE_SHININESS);

    if (numTextures == 0 || permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
    {
      params->setNamedAutoConstant("cDiffuseColour", Ogre::GpuProgramParameters::ACT_SURFACE_DIFFUSE_COLOUR);
    }
  }
  if( this->type == GBufferMaterialGenerator::GBT_DSF){
    params->setNamedAutoConstant("cObjectId",Ogre::GpuProgramParameters::ACT_CUSTOM,0);
  }

#ifdef WRITE_LINEAR_DEPTH
  //TODO : Should this be the distance to the far corner, not the far clip distance?
  params->setNamedAutoConstant("cFarDistance", Ogre::GpuProgramParameters::ACT_FAR_CLIP_DISTANCE);
#endif

  ptrProgram->load();
  return Ogre::GpuProgramPtr(ptrProgram);

}
*/
/*Ogre::MaterialPtr GBufferMaterialGeneratorImpl::GenerateTemplateMaterial(
    MaterialGenerator::Perm permutation)
{
  Ogre::String matName = this->baseName + "Mat_" + Ogre::StringConverter::toString(permutation);

  Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create
    (matName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass* pass = matPtr->getTechnique(0)->getPass(0);
  pass->setName(this->baseName + "Pass_" + Ogre::StringConverter::toString(permutation));
  pass->setLightingEnabled(false);
  if (permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
  {
    pass->createTextureUnitState();
  }
  Ogre::uint32 numTextures = permutation & GBufferMaterialGenerator::GBP_TEXTURE_MASK;
  for (Ogre::uint32 i=0; i<numTextures; i++)
  {
    pass->createTextureUnitState();
  }

  return matPtr;

}*/


