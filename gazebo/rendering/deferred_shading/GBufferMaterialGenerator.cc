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
  public: GBufferMaterialGeneratorImpl(const Ogre::String &_baseName)
          : baseName(_baseName) {}

  protected: virtual Ogre::GpuProgramPtr GenerateVertexShader(
                 MaterialGenerator::Perm permutation);
  protected: virtual Ogre::GpuProgramPtr GenerateFragmentShader(
                 MaterialGenerator::Perm permutation);
  protected: virtual Ogre::MaterialPtr GenerateTemplateMaterial(
                 MaterialGenerator::Perm permutation);

  protected: Ogre::String baseName;
};

/////////////////////////////////////////////////
GBufferMaterialGenerator::GBufferMaterialGenerator()
{
  vsMask = VS_MASK;
  fsMask = FS_MASK;
  matMask = MAT_MASK;
  materialBaseName = "DeferredShading/GBuffer/";
  this->schemeName = "GBuffer";
  this->impl = new GBufferMaterialGeneratorImpl(materialBaseName);
}

/////////////////////////////////////////////////
Ogre::GpuProgramPtr GBufferMaterialGeneratorImpl::GenerateVertexShader(
    MaterialGenerator::Perm _permutation)
{
  Ogre::StringStream ss;

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
    ss << "attribute vec3 tangentMap;\n";

  // TODO : Skinning inputs
  ss << std::endl;

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
    ss << "varying vec3 tangent;\n";
    ss << "varying vec3 biNormal;\n";
  }

  ss << std::endl;

  ss << "void main()\n{\n";
  ss << "  gl_Position = worldViewProj * gl_Vertex;" << std::endl;
  ss << "  normal = (worldView * vec4(gl_Normal,0)).xyz;" << std::endl;

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
  {
    // THis is probably wrong because I copied it incorrectly
    ss << " tangent = (worldView * vec4(tangentMap,0)).xyz;\n";
    ss << " biNormal = cross(normal, tangent);\n";
  }

#ifdef WRITE_LINEAR_DEPTH
  ss << "  viewPos = (worldView * gl_Vertex).xyz;\n";
#else
  ss << "  depth = gl_Position.w;\n";
#endif

  ss << "  gl_TexCoord[0] = gl_MultiTexCoord0;\n";
  ss << "}\n";

  // Debug output
  // std::cout << ss.str() << "\n\n";

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
  ptrProgram->setParameter("profiles", "vs_1_1 arbvp1");

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

  Ogre::uint32 numTexCoords =
    (_permutation & GBufferMaterialGenerator::GBP_TEXCOORD_MASK) >> 8;
  Ogre::uint32 numTextures =
    _permutation & GBufferMaterialGenerator::GBP_TEXTURE_MASK;

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
    ss << "uniform sampler2D normalMap;\n";

  for (Ogre::uint32 i = 0; i < numTextures; ++i)
    ss << "uniform sampler2D tex" << i << ";\n";

  if (numTextures == 0 ||
      _permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
    ss << "uniform vec4 diffuseColor;\n";

#ifdef WRITE_LINEAR_DEPTH
  ss << "uniform float farDistance;\n";
#endif

  ss << "uniform float specularity;\n";

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

  ss << std::endl;
  ss << "void main()\n{\n";

  if (numTexCoords > 0 && numTextures > 0)
  {
    ss << "  gl_FragData[0] = texture2D(tex0, gl_TexCoord[0].st);\n";
    if (_permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
      ss << "  gl_FragData[0] *= diffuseColor;\n";
  }
  else
  {
    ss << "  gl_FragData[0] = diffuseColor;\n";
  }

  ss << "  gl_FragData[0].w = specularity;\n";

  if (_permutation & GBufferMaterialGenerator::GBP_NORMAL_MAP)
  {
    ss << "  vec3 texNormal = "
       << "texture2D(normalMap, gl_TexCoord[0].st) - 0.5) * 2;\n";
    ss << "  mat3 normalRotation = mat3(tangent, biNormal, normal);\n";
    ss << "  gl_FragData[1] = normalize(texNormal * normalRotation);\n";
  }
  else
  {
    ss << "  gl_FragData[1] = vec4(normalize(normal), 0.0);\n";
  }

#ifdef WRITE_LINEAR_DEPTH
  ss << "  gl_FragData[1].w = length(viewPos) / farDistance;\n";
#else
  ss << "  gl_FragData[1].w = depth;\n";
#endif

  ss << "}\n";

  // Debug output
  std::cout << ss.str() << "\n\n";
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
  // ptrProgram->setParameter("profiles", "ps_2_0 arbfp1");

  const Ogre::GpuProgramParametersSharedPtr &params =
    ptrProgram->getDefaultParameters();

  params->setNamedAutoConstant("specularity",
      Ogre::GpuProgramParameters::ACT_SURFACE_SHININESS);

  if (numTextures == 0 ||
      _permutation & GBufferMaterialGenerator::GBP_HAS_DIFFUSE_COLOUR)
  {
    params->setNamedAutoConstant("diffuseColor",
        Ogre::GpuProgramParameters::ACT_SURFACE_DIFFUSE_COLOUR);
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

  Ogre::uint32 numTextures = _permutation &
    GBufferMaterialGenerator::GBP_TEXTURE_MASK;

  for (Ogre::uint32 i = 0; i < numTextures; ++i)
    pass->createTextureUnitState();

  return matPtr;
}
