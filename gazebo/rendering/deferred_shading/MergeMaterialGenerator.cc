/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <OgreHighLevelGpuProgram.h>
#include <OgreTechnique.h>
#include <OgreTextureUnitState.h>

#include "gazebo/rendering/deferred_shading/MergeMaterialGenerator.hh"

using namespace gazebo;
using namespace rendering;

/// This is the concrete implementation of the material generator.
class MergeMaterialGeneratorImpl : public MaterialGenerator::Impl
{
  public:
    MergeMaterialGeneratorImpl(const Ogre::String &_baseName, bool _useDSF)
      : baseName(_baseName), useDSF(_useDSF) {}

  protected: virtual Ogre::GpuProgramPtr GenerateVertexShader(
                 MaterialGenerator::Perm _permutation);
  protected: virtual Ogre::GpuProgramPtr GenerateFragmentShader(
                 MaterialGenerator::Perm _permutation);
  protected: virtual Ogre::MaterialPtr GenerateTemplateMaterial(
                 MaterialGenerator::Perm _permutation);

  protected: Ogre::String baseName;
  private: bool useDSF;
};

/////////////////////////////////////////////////
MergeMaterialGenerator::MergeMaterialGenerator(Ogre::String _matName,
    bool _useDSF)
{
  this->vsMask = VS_MASK;
  this->fsMask = FS_MASK;
  this->matMask = MAT_MASK;
  materialBaseName = _matName + "/Merge/";
  this->impl = new MergeMaterialGeneratorImpl(materialBaseName, _useDSF);
}

/////////////////////////////////////////////////
Ogre::GpuProgramPtr MergeMaterialGeneratorImpl::GenerateVertexShader(
    MaterialGenerator::Perm _permutation)
{
  Ogre::StringStream ss;

  uint32_t numTexCoords =
    (_permutation & MergeMaterialGenerator::MP_TEXCOORD_MASK) >> 8;

  ss << "uniform mat4 worldViewProj;\n";
  ss << "uniform mat4 worldView;\n";

  if (_permutation & MergeMaterialGenerator::MP_NORMAL_MAP)
  {
    ss << "attribute vec3 iTangent;\n";
    ss << "varying vec3 tangent;\n";
    ss << "varying vec3 biNormal;\n";
  }

  if (this->useDSF)
    ss << "varying vec3 viewPos;\n";

  ss << "varying vec3 normal;\n";

  ss << "void main()\n{\n";
  ss << "  gl_Position = worldViewProj * gl_Vertex;\n";
  ss << "  normal = (worldView * vec4(gl_Normal, 0.0)).xyz;\n";

  if (_permutation & MergeMaterialGenerator::MP_NORMAL_MAP )
  {
    ss << "  tangent = mul(worldView, vec4(iTangent, 0.0)).xyz;\n";
    ss << "  biNormal = cross(normal, tangent);\n";
  }

  for (uint32_t i = 0; i < numTexCoords; ++i)
    ss << "gl_TexCoord[" << i << "] = gl_MultiTexCoord" << i << ";\n";

  if (this->useDSF)
    ss << "  viewPos = (worldView * gl_Vertex).xyz;\n";

  ss << "}\n";

  Ogre::String programSource = ss.str();

  std::cout << programSource << "\n";
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
Ogre::GpuProgramPtr MergeMaterialGeneratorImpl::GenerateFragmentShader(
    MaterialGenerator::Perm _permutation)
{
  Ogre::StringStream ss;

  // comparison thresholds for Inferred Lighting. The thresholds are
  // a compromise between getting aliased edges, and getting blocky
  // illumination due to pixel-sized details offseting the comparison
  if (this->useDSF)
  {
    ss << "#define depth_epsilon 0.001\n";
    ss << "#define normal_epsilon 0.2\n";
  }

  // use the pixel position to index the LBuffer; this is more precise than
  // computing tex coords based on the view matrix(it's slightly more expensive
  // as well).
  if (this->useDSF)
  {
    ss << "uniform float farDistance;\n";
    ss << "uniform half objectId;\n";
    ss << "varying vec3 viewPos;\n";
  }


  if (_permutation & MergeMaterialGenerator::MP_NORMAL_MAP)
  {
    ss << "uniform sampler2D normalMap;\n";
    ss << "varying vec3 tanget;\n";
    ss << "varying vec3 biNormal;\n";
  }

  uint32_t numTextures = _permutation & MergeMaterialGenerator::MP_TEXTURE_MASK;
  uint32_t numTexCoords = (_permutation &
      MergeMaterialGenerator::MP_TEXCOORD_MASK) >> 8;

  for (uint32_t i = 0; i < numTextures; ++i)
    ss << "uniform sampler2D tex" << i << ";\n";

  ss << "uniform sampler2D LBuffer;\n";

  if (this->useDSF)
    ss << "uniform sampler2D DSFBuffer;\n";

  if (numTextures == 0 ||
      _permutation & MergeMaterialGenerator::MP_HAS_DIFFUSE_COLOUR)
    ss << "uniform vec4 diffuseColor;\n";

  ss << "uniform float specularity;\n";
  ss << "uniform float height;\n";
  ss << "uniform float width;\n";
  ss << "uniform float flip;\n";

  ss << "varying vec3 normal;\n";

  ss << "void main()\n{\n";

  ss << "  vec2 pixpos = gl_FragCoord.xy * vec2(1.0, -flip);\n";

  if (this->useDSF)
    ss << "  vec2 LBuffpos_frac = frac(pixpos * 0.75);\n";

  ss << "  pixpos.x /= width;\n";
  ss << "  pixpos.y /= height;\n";

  if (this->useDSF)
  {
    /// calculate sample positions in order to reconstruct bilinear filtering
    ss << "  float depth = length(viewPos) / farDistance;\n";
    ss << "  vec2 sample0 = vec2(+1.0/width, -1.0/height) / (2.0 * 0.75);\n";
    ss << "  vec2 sample1 = vec2(-1.0/width, -1.0/height) / (2.0 * 0.75);\n";
    ss << "  vec2 sample2 = vec2(+1.0/width, +1.0/height) / (2.0 * 0.75);\n";
    ss << "  vec2 sample3 = vec2(-1.0/width, +1.0/height) / (2.0 * 0.75);\n";

    // get the values of the Discontinuity Sensible Filter
    ss << "  vec4 DSF0 = texture2D(DSFBuffer, pixpos + sample0);\n";
    ss << "  vec4 DSF1 = texture2D(DSFBuffer, pixpos + sample1);\n";
    ss << "  vec4 DSF2 = texture2D(DSFBuffer, pixpos + sample2);\n";
    ss << "  vec4 DSF3 = texture2D(DSFBuffer, pixpos + sample3);\n";

    // normalize the input normal for proper comparison
    ss << "  vec3 normal = normalize(normal);\n";

    // get the usual bilinear interpolation weights
    ss << "  vec4 w = vec4((1.0 - LBuffpos_frac.x) * (1.0 - LBuffpos_frac.y),\n"
      << "      LBuffpos_frac.x * (1.0 - LBuffpos_frac.y),\n"
      << "      (1.0 - LBuffpos_frac.x) * LBuffpos_frac.y,\n"
      << "      LBuffpos_frac.x * LBuffpos_frac.y);\n";

    // see if each sample is on the same surface or not
    ss << "  float w0 =\n"
      << "clamp(100.0 * (depth_epsilon - abs(depth - DSF0.x)), 0.0, 1.0) * "
      << "clamp((1.0 - 1000.0 * abs(DSF0.y - objectId)), 0.0, 1.0) * "
      << "clamp(1000.0 * (normal_epsilon - abs(normal.x - DSF0.z)), 0.0, 1.0)*"
      << "clamp(1000.0 * (normal_epsilon - abs(normal.y - DSF0.w)), 0.0, 1.0);"
      << std::endl;

    ss <<  "  float w1 =\n"
      << "clamp(100.0 * (depth_epsilon - abs(depth - DSF1.x)), 0.0, 1.0) *"
      << "clamp((1.0 - 1000.0 * abs(DSF1.y - objectId)), 0.0, 1.0) *"
      << "clamp(1000.0 * (normal_epsilon - abs(normal.x - DSF1.z)), 0.0, 1.0)*"
      << "clamp(1000.0 * (normal_epsilon - abs(normal.y - DSF1.w)));\n";

    ss << "  float w2 =\n"
      << "clamp(100.0 * (depth_epsilon - abs(depth - DSF2.x)), 0.0, 1.0) *"
      << "clamp((1.0 - 1000.0 * abs(DSF2.y - objectId)), 0.0, 1.0) *"
      << "clamp(1000.0 * (normal_epsilon - abs(normal.x - DSF2.z))) *"
      << "clamp(1000.0 * (normal_epsilon - abs(normal.y - DSF2.w)));\n";

    ss << "  float w3 =\n"
      << "clamp(100.0 * (depth_epsilon - abs(depth - DSF3.x)),0.0, 1.0) *"
      << "clamp((1.0 - 1000.0 * abs(DSF3.y - objectId)), 0.0, 1.0) *"
      << "clamp(1000.0 * (normal_epsilon-abs(normal.x - DSF3.z)), 0.0, 1.0)*"
      << "clamp(1000.0 * (normal_epsilon - abs(normal.y - DSF3.w)));\n";

    // TODO: find a better way to check if at least a sample is on the surface
    ss <<"  if (w0 + w1 + w2 + w3 > 0)\n";
    //  bias bilinear filtering
    ss <<"    w *= vec4(w0, w1, w2, w3);\n";

    // get the light's value
    ss << "vec4 lightVal =\n"
      << " (w.x * texture2D(LBuffer, pixpos + sample0) +"
      << "  w.y * texture2D(LBuffer, pixpos + sample1) +"
      << "  w.z * texture2D(LBuffer, pixpos + sample2) +"
      << "  w.w * texture2D(LBuffer, pixpos + sample3)) / "
      << " (w.x + w.y + w.z + w.w);\n";
  }
  else
  {
    ss << "  vec4 lightVal = texture2D(LBuffer, pixpos);\n";
  }

  ss << "  vec3 diffuseCol;\n";

  if (numTexCoords > 0 && numTextures > 0)
  {
    ss << "  diffuseCol = texture2D(tex0, gl_TexCoord[0].xy).xyz;\n";
    if (_permutation & MergeMaterialGenerator::MP_HAS_DIFFUSE_COLOUR)
    {
      ss << "  diffuseCol *= diffuseColor.xyz;\n";
    }
  }
  else
  {
    ss << "  diffuseCol = diffuseColor.xyz;\n";
  }

  // use the luminance as an aproximation of the intensity of the
  // NL*attenuation term
  ss << "  float luminance = dot(lightVal.xyz,vec3(0.2126, 0.7152, 0.0722));\n";

  // reconstruct HV^n_light based on that, and get the specular term
  ss << "  vec3 specular = diffuseCol * max(0.0, pow(lightVal.w / luminance, "
    << "1.0 + specularity));\n";

  // get the final lighting value
  ss << "  gl_FragColor = vec4(lightVal.xyz * (diffuseCol + specular), 1.0);\n";
  ss << "}\n";

  Ogre::String programSource = ss.str();
  Ogre::String programName = this->baseName + "FP_" +
    Ogre::StringConverter::toString(_permutation);

  std::cout << programSource << "\n";
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

  uint32_t texIndex = 0;
  for (; texIndex < numTextures; ++texIndex)
  {
    std::ostringstream stream;
    stream << "tex" << texIndex;
    params->setNamedConstant(stream.str(), static_cast<int>(texIndex));
  }

  params->setNamedConstant("LBuffer", static_cast<int>(texIndex));

  if (numTextures == 0 || _permutation &
      MergeMaterialGenerator::MP_HAS_DIFFUSE_COLOUR)
  {
    params->setNamedAutoConstant("diffuseColor",
        Ogre::GpuProgramParameters::ACT_SURFACE_DIFFUSE_COLOUR);
  }

  params->setNamedAutoConstant("specularity",
      Ogre::GpuProgramParameters::ACT_SURFACE_SHININESS);
  // params->setNamedAutoConstant("specularColor",
  // Ogre::GpuProgramParameters::ACT_SURFACE_SPECULAR_COLOUR);
  params->setNamedAutoConstant("width",
      Ogre::GpuProgramParameters::ACT_VIEWPORT_WIDTH);
  params->setNamedAutoConstant("height",
      Ogre::GpuProgramParameters::ACT_VIEWPORT_HEIGHT);
  params->setNamedAutoConstant("flip",
      Ogre::GpuProgramParameters::ACT_RENDER_TARGET_FLIPPING);

  if (this->useDSF)
  {
    params->setNamedAutoConstant("farDistance",
        Ogre::GpuProgramParameters::ACT_FAR_CLIP_DISTANCE);
    params->setNamedAutoConstant("objectId",
        Ogre::GpuProgramParameters::ACT_CUSTOM, 0);
  }

  ptrProgram->load();

  return Ogre::GpuProgramPtr(ptrProgram);
}

/////////////////////////////////////////////////
Ogre::MaterialPtr MergeMaterialGeneratorImpl::GenerateTemplateMaterial(
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

  if (_permutation & MergeMaterialGenerator::MP_NORMAL_MAP)
    pass->createTextureUnitState();

  uint32_t numTextures = _permutation &
    MergeMaterialGenerator::MP_TEXTURE_MASK;

  for (uint32_t i = 0; i < numTextures + 1; ++i)
    pass->createTextureUnitState();

  if (this->useDSF)
    pass->createTextureUnitState();

  return matPtr;
}

//////////////////////////////////////////////////////////////////////////////
// CG shader code taken from Ogre Deferred Shading sample. DELETE ME
/*
   Ogre::GpuProgramPtr MergeMaterialGeneratorImpl::GenerateVertexShader(
   MaterialGenerator::Perm permutation)
   {
   Ogre::StringStream ss;

   ss << "void MergeVP(" << std::endl;
   ss << "  float4 iPosition : POSITION," << std::endl;
//Get the normal, although this isn't 100% necesarry, it is cheaper to use the normal from the input
//geometry than fetching it from the GBuffer
ss << "  float3 iNormal   : NORMAL," << std::endl;

Ogre::uint32 numTexCoords = (permutation & MergeMaterialGenerator::MP_TEXCOORD_MASK) >> 8;
for (Ogre::uint32 i = 0; i < numTexCoords; i++)
{
ss << "  float2 iUV" << i << " : TEXCOORD" << i << ',' << std::endl;
}

if (permutation & MergeMaterialGenerator::MP_NORMAL_MAP)
{
ss << "  float3 iTangent : TANGENT0," << std::endl;
}

//TODO : Skinning inputs
ss << std::endl;

ss << "  out float4 oPosition : POSITION," << std::endl;

int texCoordNum=0;
ss << "  out half3 oNormal : TEXCOORD" <<texCoordNum++<<","<<std::endl;

if ((permutation & MergeMaterialGenerator::MP_NORMAL_MAP) )
{
ss << "  out half3 oTangent : TEXCOORD" << texCoordNum++ << ',' << std::endl;
ss << "  out half3 oBiNormal : TEXCOORD" << texCoordNum++ << ',' << std::endl;
}
for (Ogre::uint32 i = 0; i<numTexCoords; i++)
{
ss << "  out half2 oUV" << i << " : TEXCOORD" << texCoordNum++ << ',' << std::endl;
}
if(this->useDSF){
ss << "  out float3 oViewPos : TEXCOORD"<<texCoordNum++<<"," << std::endl;
}
ss << std::endl;

ss << "  uniform float4x4 cWorldViewProj," << std::endl;
ss << "  uniform float4x4 cWorldView" << std::endl;

ss << "  )" << std::endl;


ss << "{" << std::endl;
ss << "  oPosition = mul(cWorldViewProj, iPosition);" << std::endl;
ss << "  oNormal = mul(cWorldView, float4(iNormal,0)).xyz;" << std::endl;

if (permutation & MergeMaterialGenerator::MP_NORMAL_MAP)
{
ss << "  oTangent = mul(cWorldView, float4(iTangent,0)).xyz;" << std::endl;
ss << "  oBiNormal = cross(oNormal, oTangent);" << std::endl;
}

for (Ogre::uint32 i = 0; i < numTexCoords; i++) {
ss << "  oUV" << i << " = iUV" << i << ';' << std::endl;
}
if(this->useDSF){
ss << "  oViewPos = mul(cWorldView, iPosition).xyz;" << std::endl;
}
ss << "}" << std::endl;

Ogre::String programSource = ss.str();

Ogre::String programName = this->baseName + "VP_" + Ogre::StringConverter::toString(permutation);
std::cout << programSource << "\n";
//#if OGRE_DEBUG_MODE
Ogre::LogManager::getSingleton().getDefaultLog()->logMessage(programSource);
//#endif

// Create shader object
Ogre::HighLevelGpuProgramPtr ptrProgram = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
    programName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    "cg", Ogre::GPT_VERTEX_PROGRAM);
ptrProgram->setSource(programSource);
ptrProgram->setParameter("entry_point","MergeVP");
ptrProgram->setParameter("profiles","vs_3_0 arbvp1");

const Ogre::GpuProgramParametersSharedPtr& params = ptrProgram->getDefaultParameters();
params->setNamedAutoConstant("cWorldViewProj", Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
params->setNamedAutoConstant("cWorldView", Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX);
ptrProgram->load();

return Ogre::GpuProgramPtr(ptrProgram);
}

Ogre::GpuProgramPtr MergeMaterialGeneratorImpl::GenerateFragmentShader(
    MaterialGenerator::Perm permutation)
{
  Ogre::StringStream ss;

  //DirectX needs this to compensate for jitter

  ss <<"ps_3_0 float2 fixUV(float2 texCoord,float flip){"<<std::endl;
  ss <<"return (texCoord*float2(1,flip) + float2(0.5,0.5));}"<<std::endl;

  ss<<"float2 fixUV(float2 texCoord,float flip){"<<std::endl;
  ss<<"return texCoord*float2(1,-flip);}"<<std::endl;
  //comparison thresholds for Inferred Lighting. The thresholds are a compromise between getting aliased edges,
  //and getting blocky illumination due to pixel-sized details offseting the comparison

  if(this->useDSF){
    ss << "#define depth_epsilon 0.001" << std::endl;
    ss << "#define normal_epsilon 0.2" << std::endl;
  }
  ss << "void MergeFP(" << std::endl;
  ss << "  half3 iNormal : TEXCOORD0," <<std::endl;
  //use the pixel position to index the LBuffer; this is more precise than
  //computing tex coords based on the view matrix(it's slightly more expensive
  //as well).
  ss << "  float2 pixpos: WPOS";
  if(this->useDSF){
    ss<< ","<<std::endl<<"  uniform float cFarDistance";
    ss<< ","<<std::endl<<"  uniform half cObjectId";
  }
  int texCoordNum = 1;
  if (permutation & MergeMaterialGenerator::MP_NORMAL_MAP )
  {
    ss << ',' << std::endl << "  half3 iTangent : TEXCOORD" << texCoordNum++ ;
    ss << ',' << std::endl << "  half3 iBiNormal : TEXCOORD" << texCoordNum++ ;
  }

  Ogre::uint32 numTexCoords = (permutation & MergeMaterialGenerator::MP_TEXCOORD_MASK) >> 8;
  for (Ogre::uint32 i = 0; i < numTexCoords; i++)
  {
    ss << ',' << std::endl<< "  half2 iUV" << i << " : TEXCOORD" << texCoordNum++ ;
  }
  if(this->useDSF)
    ss << ',' << std::endl<< "  float3 iViewPos: TEXCOORD" << texCoordNum++ ;

  ss << ',' << std::endl<< "  out half4 oColor0 : COLOR0" ;

  int samplerNum = 0;

  if (permutation & MergeMaterialGenerator::MP_NORMAL_MAP)
  {
    ss << "," << std::endl << "  uniform sampler sNormalMap : register(s" << samplerNum++ << ")";
  }

  Ogre::uint32 numTextures = permutation & MergeMaterialGenerator::MP_TEXTURE_MASK;


  for (Ogre::uint32 i = 0; i<numTextures; i++) {
    ss << "," <<std::endl << "  uniform sampler sTex" << i << " : register(s" << samplerNum++ << ")";
  }
  ss << "," <<std::endl<<"  uniform sampler LBuffer : register(s"<< samplerNum++ <<")";
  if(this->useDSF){
    ss << "," <<std::endl<<"  uniform sampler DSFBuffer : register(s"<< samplerNum++ <<")";
  }
  if (numTextures == 0 || permutation & MergeMaterialGenerator::MP_HAS_DIFFUSE_COLOUR)
  {
    ss<<","<< std::endl << "  uniform half4 cDiffuseColour" ;
  }
  ss <<","<<std::endl<< "  uniform half cSpecularity" ;
  ss <<","<<std::endl<< "  uniform half cHeight";
  ss <<","<<std::endl<< "  uniform half cWidth";
  ss <<","<<std::endl<< "  uniform half cFlip";
  ss << "  )" << std::endl;


  ss << "{" << std::endl;
  ss << " pixpos = fixUV(pixpos,cFlip);"<<std::endl;
  if(this->useDSF)
    ss <<"  float2 LBuffpos_frac = frac(pixpos*0.75);"<<std::endl;
  //ss <<"  pixpos+=float2(0.5/0.75,0.5/0.75);"<<std::endl;
  ss <<"  pixpos.x/=cWidth;"<<std::endl;
  ss <<"  pixpos.y/=cHeight;"<<std::endl;
  if(this->useDSF){
    //calculate sample positions in order to reconstruct bilinear filtering
    ss <<"  float depth = length(iViewPos) / cFarDistance;"<<std::endl;
    ss <<"  float2 sample0 = float2(1/cWidth,- 1/cHeight) /(2*0.75);"<<std::endl;
    ss <<"  float2 sample1 = float2(-1/cWidth,- 1/cHeight) /(2*0.75);"<<std::endl;
    ss <<"  float2 sample2 = float2(1/cWidth ,+ 1/cHeight )/(2*0.75);"<<std::endl;
    ss <<"  float2 sample3 = float2(-1/cWidth,+ 1/cHeight) /(2*0.75);"<<std::endl;


    //get the values of the Discontinuity Sensible Filter
    ss <<"  float4 DSF0 = tex2D(DSFBuffer,pixpos + sample0);"<<std::endl;
    ss <<"  float4 DSF1 = tex2D(DSFBuffer,pixpos + sample1);"<<std::endl;
    ss <<"  float4 DSF2 = tex2D(DSFBuffer,pixpos + sample2);"<<std::endl;
    ss <<"  float4 DSF3 = tex2D(DSFBuffer,pixpos + sample3);"<<std::endl;
    //normalize the input normal for proper comparison
    ss <<"  iNormal=normalize(iNormal);"<<std::endl;

    //get the usual bilinear interpolation weights
    ss <<"  float4 w = float4((1-LBuffpos_frac.x)*(1-LBuffpos_frac.y),LBuffpos_frac.x*(1-LBuffpos_frac.y),"<<std::endl
      <<"(1-LBuffpos_frac.x)*LBuffpos_frac.y,LBuffpos_frac.x*LBuffpos_frac.y);"<<std::endl;

    //see if each sample is on the same surface or not
    ss <<"  float w0 = saturate(100*(depth_epsilon - abs(depth-DSF0.r))) *"<<std::endl
      <<"    saturate((1-1000*abs(DSF0.g - cObjectId)))*saturate(1000*(normal_epsilon - abs(iNormal.x-DSF0.b)))*saturate(1000*(normal_epsilon - abs(iNormal.y-DSF0.a)));"<<std::endl;
    ss <<"  float w1 = saturate(100*(depth_epsilon - abs(depth-DSF1.r))) *"<<std::endl
      <<"    saturate((1-1000*abs(DSF1.g - cObjectId)))*saturate(1000*(normal_epsilon - abs(iNormal.x-DSF1.b)))*saturate(1000*(normal_epsilon - abs(iNormal.y-DSF1.a)));"<<std::endl;
    ss <<"  float w2 = saturate(100*(depth_epsilon - abs(depth-DSF2.r))) *"<<std::endl
      <<"    saturate((1-1000*abs(DSF2.g - cObjectId)))*saturate(1000*(normal_epsilon - abs(iNormal.x-DSF2.b)))*saturate(1000*(normal_epsilon - abs(iNormal.y-DSF2.a)));"<<std::endl;
    ss <<"  float w3 = saturate(100*(depth_epsilon - abs(depth-DSF3.r))) *"<<std::endl
      <<"    saturate((1-1000*abs(DSF3.g - cObjectId)))*saturate(1000*(normal_epsilon - abs(iNormal.x-DSF3.b)))*saturate(1000*(normal_epsilon - abs(iNormal.y-DSF3.a)));"<<std::endl;
    //TODO: find a better way to check if at least a sample is on the surface
    ss <<" if(w0+w1+w2+w3 > 0)"<<std::endl;
    //bias bilinear filtering
    ss <<" w *= float4(w0,w1,w2,w3);"<<std::endl;
    //get the light's value
    ss <<"half4 lightVal = (w.x * tex2D(LBuffer,pixpos+sample0) + w.y * tex2D(LBuffer,pixpos + sample1) +w.z * tex2D(LBuffer,pixpos + sample2)+w.w * tex2D(LBuffer,pixpos + sample3))/(w.x+w.y+w.z+w.w);"<<std::endl;
  }
  else{
    ss <<"  half4 lightVal = tex2D(LBuffer,pixpos);"<<std::endl;
  }

  ss << "  half3 diffuseCol;"<<std::endl;
  if (numTexCoords > 0 && numTextures > 0 )
  {
    ss << "  diffuseCol = tex2D(sTex0, iUV0);" << std::endl;
    if (permutation & MergeMaterialGenerator::MP_HAS_DIFFUSE_COLOUR)
    {
      ss << "  diffuseCol *= cDiffuseColour.rgb;" << std::endl;
    }
  }
  else
  {
    ss << "  diffuseCol = cDiffuseColour.rgb;" << std::endl;
  }
  //use the luminance as an aproximation of the intensity of the NL*attenuation term
  ss <<"  half luminance = dot(lightVal.rgb, float3(0.2126, 0.7152, 0.0722));"<<std::endl;
  //reconstruct HV^n_light based on that, and get the specular term
  ss <<"  half3 specular = diffuseCol*max(0,pow(lightVal.a/luminance,1.0+cSpecularity));"<<std::endl;
  //get the final lighting value
  ss <<"  half4 color = float4(lightVal.rgb*(diffuseCol + specular),1);"<<std::endl;
  ss <<"  oColor0 = color;"<<std::endl;
  ss << "}" << std::endl;

  Ogre::String programSource = ss.str();
  Ogre::String programName = this->baseName + "FP_" + Ogre::StringConverter::toString(permutation);

  std::cout << programSource << "\n";
  //#if OGRE_DEBUG_MODE
  Ogre::LogManager::getSingleton().getDefaultLog()->logMessage(programSource);
  //#endif

  // Create shader object
  Ogre::HighLevelGpuProgramPtr ptrProgram = Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
      programName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      "cg", Ogre::GPT_FRAGMENT_PROGRAM);
  ptrProgram->setSource(programSource);
  ptrProgram->setParameter("entry_point","MergeFP");
  ptrProgram->setParameter("profiles","ps_3_0 arbfp1");

  const Ogre::GpuProgramParametersSharedPtr& params = ptrProgram->getDefaultParameters();


  if (numTextures == 0 || permutation & MergeMaterialGenerator::MP_HAS_DIFFUSE_COLOUR)
  {
    params->setNamedAutoConstant("cDiffuseColour", Ogre::GpuProgramParameters::ACT_SURFACE_DIFFUSE_COLOUR);
  }
  params->setNamedAutoConstant("cSpecularity", Ogre::GpuProgramParameters::ACT_SURFACE_SHININESS);
  //params->setNamedAutoConstant("cSpecularColour", Ogre::GpuProgramParameters::ACT_SURFACE_SPECULAR_COLOUR);
  params->setNamedAutoConstant("cWidth", Ogre::GpuProgramParameters::ACT_VIEWPORT_WIDTH);
  params->setNamedAutoConstant("cHeight", Ogre::GpuProgramParameters::ACT_VIEWPORT_HEIGHT);
  params->setNamedAutoConstant("cFlip", Ogre::GpuProgramParameters::ACT_RENDER_TARGET_FLIPPING);
  if(this->useDSF){
    params->setNamedAutoConstant("cFarDistance", Ogre::GpuProgramParameters::ACT_FAR_CLIP_DISTANCE);
    params->setNamedAutoConstant("cObjectId",Ogre::GpuProgramParameters::ACT_CUSTOM,0);
  }
  ptrProgram->load();
  return Ogre::GpuProgramPtr(ptrProgram);
}*/
