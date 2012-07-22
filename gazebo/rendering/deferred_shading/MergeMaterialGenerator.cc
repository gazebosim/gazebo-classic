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
  uint32_t numTexCoords =
    (_permutation & MergeMaterialGenerator::MP_TEXCOORD_MASK) >> 8;
  int texCoordNum=0;

  Ogre::StringStream ss;

  ss << "uniform mat4 worldViewProj;\n";
  ss << "uniform mat4 worldView;\n";

  if (_permutation & MergeMaterialGenerator::MP_NORMAL_MAP)
    ss << "  attribute vec3 tangent;\n";

  if (this->useDSF)
    ss << "  varying vec3 viewPos;\n";

  ss << "void main()\n{\n";
  ss << "  gl_Position = worldViewProj * gl_Vertex;\n";
  ss << "  gl_TexCoord[0].xyz = (worldView * vec4(gl_Normal, 0.0)).xyz;\n";
  // ss << "  vec4 projPos = worldViewProj * gl_Vertex;\n";

  texCoordNum = 1;
  if (_permutation & MergeMaterialGenerator::MP_NORMAL_MAP )
  {
    ss << "  gl_TexCoord[1] = mul(worldView, vec4(tangent, 0.0)).xyz;\n";
    ss << "  gl_TexCoord[2] = cross(gl_TexCoord[0], gl_TexCoord[1]);\n";
    texCoordNum = 3;
  }

  for (uint32_t i = 0; i < numTexCoords; ++i)
    ss << "gl_TexCoord[" << i + texCoordNum << "] = gl_MultiTexCoord"
       << i + texCoordNum << ";\n";

  if (this->useDSF)
    ss << "  viewPos = (worldView * gl_Vertex).xyz;\n";

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
  ptrProgram->setParameter("profiles","vs_3_0 arbvp1");

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

  //use the pixel position to index the LBuffer; this is more precise than
  //computing tex coords based on the view matrix(it's slightly more expensive
  //as well). 
  if (this->useDSF)
  {
    ss << "uniform float farDistance;\n";
    ss << "uniform half objectId;\n";
    ss << "varying vec3 viewPos;\n";
  }

  ss << "uniform sampler2D LBuffer;\n";

  uint32_t numTextures = _permutation & MergeMaterialGenerator::MP_TEXTURE_MASK;
  uint32_t numTexCoords = (_permutation &
      MergeMaterialGenerator::MP_TEXCOORD_MASK) >> 8;

  for (uint32_t i = 0; i < numTextures; ++i)
    ss << "uniform sampler2D sTex" << i << ";\n";

  if (this->useDSF)
    ss << "uniform sampler2D DSFBuffer;\n";

  if (numTextures == 0 ||
      _permutation & MergeMaterialGenerator::MP_HAS_DIFFUSE_COLOUR)
    ss << "uniform vec4 diffuseColor;\n";

  ss << "uniform float specularity;\n";
  ss << "uniform float height;\n";
  ss << "uniform float width;\n";
  ss << "uniform float flip;\n";

  ss << "void main()\n{\n";

  ss << "  vec2 pixpos = gl_FragCoord.xy * vec2(1.0, -flip);\n";

  if(this->useDSF)
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
    ss << "  vec3 normal = normalize(gl_TexCoord[0].xyz);\n";

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
    ss << "  diffuseCol = texture2D(sTex0, gl_TexCoord[0].xy).xyz;\n";
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
  ss << "  vec4 color = vec4(lightVal.xyz * (diffuseCol + specular), 1.0);\n";

  ss <<"	gl_FragColor = color;\n";
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
  ptrProgram->setParameter("profiles","ps_3_0 arbfp1");

  const Ogre::GpuProgramParametersSharedPtr &params =
    ptrProgram->getDefaultParameters();

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

  if(this->useDSF)
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
