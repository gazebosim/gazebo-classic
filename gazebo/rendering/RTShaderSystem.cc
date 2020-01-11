/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <sys/stat.h>
#include <boost/filesystem.hpp>

#if defined(HAVE_OPENGL)

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#else
#if defined(_WIN32)
  #include <windows.h>
#endif /* _WIN32 */
#include <GL/gl.h>
#include <GL/glext.h>
#endif /* __APPLE__ */

#endif /* HAVE_OPENGL */


#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/CustomPSSMShadowCameraSetup.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/RTShaderSystemPrivate.hh"
#include "gazebo/rendering/RTShaderSystem.hh"

#define MINOR_VERSION 7
using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
RTShaderSystem::RTShaderSystem()
  : dataPtr(new RTShaderSystemPrivate)
{
  this->dataPtr->initialized = false;
  this->dataPtr->shadowsApplied = false;
  this->dataPtr->pssmSetup.setNull();
  this->dataPtr->updateShaders = false;
}

//////////////////////////////////////////////////
RTShaderSystem::~RTShaderSystem()
{
  this->Fini();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
void RTShaderSystem::Init()
{
#if INCLUDE_RTSHADER && OGRE_VERSION_MAJOR >= 1 &&\
    OGRE_VERSION_MINOR >= MINOR_VERSION

  // Only initialize if using FORWARD rendering
  if (RenderEngine::Instance()->GetRenderPathType() != RenderEngine::FORWARD)
    return;

  if (Ogre::RTShader::ShaderGenerator::initialize())
  {
    this->dataPtr->initialized = true;

    std::string coreLibsPath, cachePath;
    this->GetPaths(coreLibsPath, cachePath);

    // Get the shader generator pointer
    this->dataPtr->shaderGenerator =
        Ogre::RTShader::ShaderGenerator::getSingletonPtr();

    // Add the shader libs resource location
    coreLibsPath = boost::filesystem::path(coreLibsPath)
        .make_preferred().string();
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        coreLibsPath, "FileSystem");

    // Set shader cache path.
    this->dataPtr->shaderGenerator->setShaderCachePath(cachePath);

#if OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR <= 8
    this->dataPtr->programWriterFactory =
        OGRE_NEW CustomGLSLProgramWriterFactory();
    Ogre::RTShader::ProgramWriterManager::getSingletonPtr()->addFactory(
        this->dataPtr->programWriterFactory);
#endif

    this->dataPtr->shaderGenerator->setTargetLanguage("glsl");

    Ogre::RTShader::SubRenderStateFactory* factory =
        OGRE_NEW CustomPSSM3Factory;
    this->dataPtr->shaderGenerator->addSubRenderStateFactory(factory);
  }
  else
    gzerr << "RT Shader system failed to initialize\n";
#endif

  // normal map is not working with the shaders in media/rtshaderlib
  // (GLSL < 130), so disable it for now.
  // this mainly affects gazebo on OSX
  const Ogre::RenderSystemCapabilities *capabilities =
      Ogre::Root::getSingleton().getRenderSystem()->getCapabilities();
  Ogre::DriverVersion glVersion;
  glVersion.build = 0;
  glVersion.major = 3;
  glVersion.minor = 0;
  glVersion.release = 0;
  if (capabilities->isDriverOlderThanVersion(glVersion))
    this->dataPtr->enableNormalMap = false;
}

//////////////////////////////////////////////////
void RTShaderSystem::Fini()
{
  if (!this->dataPtr->initialized)
    return;

  // Restore default scheme.
  Ogre::MaterialManager::getSingleton().setActiveScheme(
      Ogre::MaterialManager::DEFAULT_SCHEME_NAME);

  // Finalize RTShader system.
  if (this->dataPtr->shaderGenerator != NULL)
  {
#if (OGRE_VERSION < ((1 << 16) | (9 << 8) | 0))
    Ogre::RTShader::ShaderGenerator::finalize();
#else
    Ogre::RTShader::ShaderGenerator::destroy();
#endif

    if (this->dataPtr->programWriterFactory)
      delete this->dataPtr->programWriterFactory;

    this->dataPtr->shaderGenerator = NULL;
  }

  this->dataPtr->pssmSetup.setNull();
  this->dataPtr->scenes.clear();
  this->dataPtr->shadowsApplied = false;
  this->dataPtr->initialized = false;
}

#if INCLUDE_RTSHADER && OGRE_VERSION_MAJOR >= 1 &&\
    OGRE_VERSION_MINOR >= MINOR_VERSION
//////////////////////////////////////////////////
void RTShaderSystem::AddScene(ScenePtr _scene)
{
  if (!this->dataPtr->initialized)
    return;

  // Set the scene manager
  this->dataPtr->shaderGenerator->addSceneManager(_scene->OgreSceneManager());
  this->dataPtr->shaderGenerator->createScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
  this->dataPtr->scenes.push_back(_scene);
}
#else
void RTShaderSystem::AddScene(ScenePtr /*_scene*/)
{
}
#endif

//////////////////////////////////////////////////
void RTShaderSystem::RemoveScene(ScenePtr _scene)
{
  if (!this->dataPtr->initialized)
    return;

  auto iter = this->dataPtr->scenes.begin();
  for (; iter != this->dataPtr->scenes.end(); ++iter)
    if ((*iter) == _scene)
      break;

  if (iter != this->dataPtr->scenes.end())
  {
    this->dataPtr->scenes.erase(iter);
    this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
    this->dataPtr->shaderGenerator->removeSceneManager(
        _scene->OgreSceneManager());
    this->dataPtr->shaderGenerator->removeAllShaderBasedTechniques();
    this->dataPtr->shaderGenerator->flushShaderCache();
  }
}

//////////////////////////////////////////////////
void RTShaderSystem::RemoveScene(const std::string &_scene)
{
  if (!this->dataPtr->initialized)
    return;

  for (auto iter : this->dataPtr->scenes)
  {
    if (iter->Name() == _scene)
    {
      this->RemoveScene(iter);
      return;
    }
  }
}

//////////////////////////////////////////////////
void RTShaderSystem::AttachViewport(Ogre::Viewport *_viewport, ScenePtr _scene)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  _viewport->setMaterialScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
#endif
}

//////////////////////////////////////////////////
void RTShaderSystem::DetachViewport(Ogre::Viewport *_viewport, ScenePtr _scene)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  if (_viewport && _scene && _scene->Initialized())
    _viewport->setMaterialScheme(_scene->Name());
#endif
}

//////////////////////////////////////////////////
void RTShaderSystem::UpdateShaders()
{
  // shaders will be updated in the Update call on pre-render event.
  this->dataPtr->updateShaders = true;
}

//////////////////////////////////////////////////
void RTShaderSystem::UpdateShaders(VisualPtr _vis)
{
  if (_vis)
  {
    if (_vis->UseRTShader())
      this->GenerateShaders(_vis);
    for (unsigned int i = 0; i < _vis->GetChildCount(); ++i)
      this->UpdateShaders(_vis->GetChild(i));
  }
}

//////////////////////////////////////////////////
void RTShaderSystem::GenerateShaders(const VisualPtr &_vis)
{
  if (!this->dataPtr->initialized || !_vis)
    return;

  for (unsigned int k = 0; _vis->GetSceneNode() &&
      k < _vis->GetSceneNode()->numAttachedObjects(); ++k)
  {
    Ogre::MovableObject *obj = _vis->GetSceneNode()->getAttachedObject(k);
    Ogre::Entity *entity = dynamic_cast<Ogre::Entity*>(obj);
    if (!entity)
      continue;

    for (unsigned int i = 0; i < entity->getNumSubEntities(); ++i)
    {
      Ogre::SubEntity* curSubEntity = entity->getSubEntity(i);
      const Ogre::String& curMaterialName = curSubEntity->getMaterialName();
      bool success = false;

      for (unsigned int s = 0; s < this->dataPtr->scenes.size(); s++)
      {
        try
        {
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 10
          const Ogre::MaterialPtr& curMaterialPtr = curSubEntity->getMaterial();
          success = this->dataPtr->shaderGenerator->createShaderBasedTechnique(
              *curMaterialPtr,
#else
          success = this->dataPtr->shaderGenerator->createShaderBasedTechnique(
              curMaterialName,
#endif
              Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
              this->dataPtr->scenes[s]->Name() +
              Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
        }
        catch(Ogre::Exception &e)
        {
          gzerr << "Unable to create shader technique for material["
            << curMaterialName << "]\n";
          success = false;
        }

        // Setup custom shader sub render states according to current setup.
        if (success)
        {
          Ogre::MaterialPtr curMaterial =
            Ogre::MaterialManager::getSingleton().getByName(curMaterialName);

          // Grab the first pass render state.
          // NOTE:For more complicated samples iterate over the passes and build
          // each one of them as desired.
          Ogre::RTShader::RenderState* renderState =
            this->dataPtr->shaderGenerator->getRenderState(
                this->dataPtr->scenes[s]->Name() +
                Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME,
                curMaterialName,
                Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME,
                0);

          // Remove all sub render states.
          renderState->reset();

          if (this->dataPtr->enableNormalMap &&
              _vis->GetShaderType() == "normal_map_object_space")
          {
            Ogre::RTShader::SubRenderState* subRenderState =
              this->dataPtr->shaderGenerator->createSubRenderState(
                  Ogre::RTShader::NormalMapLighting::Type);

            Ogre::RTShader::NormalMapLighting* normalMapSubRS =
              static_cast<Ogre::RTShader::NormalMapLighting*>(subRenderState);

            normalMapSubRS->setNormalMapSpace(
                Ogre::RTShader::NormalMapLighting::NMS_OBJECT);

            normalMapSubRS->setNormalMapTextureName(_vis->GetNormalMap());
            renderState->addTemplateSubRenderState(normalMapSubRS);
          }
          else if (this->dataPtr->enableNormalMap &&
              _vis->GetShaderType() == "normal_map_tangent_space")
          {
            Ogre::RTShader::SubRenderState* subRenderState =
              this->dataPtr->shaderGenerator->createSubRenderState(
                  Ogre::RTShader::NormalMapLighting::Type);

            Ogre::RTShader::NormalMapLighting* normalMapSubRS =
              static_cast<Ogre::RTShader::NormalMapLighting*>(subRenderState);

            normalMapSubRS->setNormalMapSpace(
                Ogre::RTShader::NormalMapLighting::NMS_TANGENT);

            normalMapSubRS->setNormalMapTextureName(_vis->GetNormalMap());

            renderState->addTemplateSubRenderState(normalMapSubRS);
          }
          else if (_vis->GetShaderType() == "vertex")
          {
            Ogre::RTShader::SubRenderState *perPerVertexLightModel =
              this->dataPtr->shaderGenerator->createSubRenderState(
                  Ogre::RTShader::FFPLighting::Type);

            renderState->addTemplateSubRenderState(perPerVertexLightModel);
          }
          else
          {
            Ogre::RTShader::SubRenderState *perPixelLightModel =
              this->dataPtr->shaderGenerator->createSubRenderState(
                  Ogre::RTShader::PerPixelLighting::Type);

            renderState->addTemplateSubRenderState(perPixelLightModel);
          }


          // Invalidate this material in order to re-generate its shaders.
          this->dataPtr->shaderGenerator->invalidateMaterial(
              this->dataPtr->scenes[s]->Name() +
              Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME,
              curMaterialName);
        }
      }
    }
  }
}

//////////////////////////////////////////////////
bool RTShaderSystem::GetPaths(std::string &coreLibsPath, std::string &cachePath)
{
  if (!this->dataPtr->initialized)
    return false;

  Ogre::StringVector groupVector;

  // Setup the core libraries and shader cache path
  groupVector = Ogre::ResourceGroupManager::getSingleton().getResourceGroups();
  Ogre::StringVector::iterator itGroup = groupVector.begin();
  Ogre::StringVector::iterator itGroupEnd = groupVector.end();
  Ogre::String shaderCoreLibsPath;

  for (; itGroup != itGroupEnd; ++itGroup)
  {
    Ogre::ResourceGroupManager::LocationList resLocationsList;
    Ogre::ResourceGroupManager::LocationList::iterator it;
    Ogre::ResourceGroupManager::LocationList::iterator itEnd;
    bool coreLibsFound = false;

    resLocationsList =
      Ogre::ResourceGroupManager::getSingleton().getResourceLocationList(
          *itGroup);
    it = resLocationsList.begin();
    itEnd = resLocationsList.end();
    // Try to find the location of the core shader lib functions and use it
    // as shader cache path as well - this will reduce the number of
    // generated files when running from different directories.

    for (; it != itEnd; ++it)
    {
      struct stat st;
      if (stat((*it)->archive->getName().c_str(), &st) == 0)
      {
        if ((*it)->archive->getName().find("rtshaderlib") != Ogre::String::npos)
        {
          coreLibsPath = (*it)->archive->getName() + "/";

          // setup patch name for rt shader cache in tmp
          char *tmpdir;
          char *user;
          std::ostringstream stream;
          std::ostringstream errStream;
          // Get the tmp dir
          tmpdir = getenv("TMP");
          if (!tmpdir)
          {
            common::SystemPaths *paths = common::SystemPaths::Instance();
            tmpdir = const_cast<char*>(paths->TmpPath().c_str());
          }
          // Get the user
          user = getenv("USER");
          if (!user)
            user = const_cast<char*>("nobody");
          stream << tmpdir << "/gazebo-" << user << "-rtshaderlibcache" << "/";
          cachePath = stream.str();
          // Create the directory
#ifdef _WIN32
          if (_mkdir(cachePath.c_str()) != 0)
#else
          if (mkdir(cachePath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR) != 0)
#endif
          {
            if (errno != EEXIST)
            {
              errStream << "failed to create [" << cachePath << "] : ["
                <<  strerror(errno) << "]";
              throw(errStream.str());
            }
          }

          coreLibsFound = true;
          break;
        }
      }
    }

    // Core libs path found in the current group.
    if (coreLibsFound)
      break;
  }

  // Core shader lib not found -> shader generating will fail.
  if (coreLibsPath.empty())
  {
    gzerr << "Unable to find shader lib. Shader generating will fail.";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void RTShaderSystem::RemoveShadows(ScenePtr _scene)
{
  if (!this->dataPtr->initialized || !this->dataPtr->shadowsApplied)
    return;

  _scene->OgreSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  _scene->OgreSceneManager()->setShadowCameraSetup(
      Ogre::ShadowCameraSetupPtr());

  Ogre::RTShader::RenderState* schemeRenderState =
    this->dataPtr->shaderGenerator->getRenderState(
        _scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  schemeRenderState->removeTemplateSubRenderState(
      this->dataPtr->shadowRenderState);

  this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
  this->UpdateShaders();

  this->dataPtr->shadowsApplied = false;
}

/////////////////////////////////////////////////
void RTShaderSystem::ApplyShadows(ScenePtr _scene)
{
  if (!this->dataPtr->initialized || this->dataPtr->shadowsApplied)
    return;

  Ogre::SceneManager *sceneMgr = _scene->OgreSceneManager();

  // Grab the scheme render state.
  Ogre::RTShader::RenderState* schemRenderState =
    this->dataPtr->shaderGenerator->getRenderState(_scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE_INTEGRATED);

  // 3 textures per directional light
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_DIRECTIONAL, 3);
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_POINT, 0);
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_SPOTLIGHT, 0);
  sceneMgr->setShadowTextureCount(3);

  unsigned int texSize = this->dataPtr->shadowTextureSize;
#if defined(__APPLE__)
  // workaround a weird but on OSX if texture size at 2 and 3 splits are not
  // halved
  texSize = this->dataPtr->shadowTextureSize/2;
#endif
  sceneMgr->setShadowTextureConfig(0,
      this->dataPtr->shadowTextureSize, this->dataPtr->shadowTextureSize,
      Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureConfig(1, texSize, texSize, Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureConfig(2, texSize, texSize, Ogre::PF_FLOAT32_R);

#if defined(HAVE_OPENGL)
  // Enable shadow map comparison, so shader can use
  // float texture(sampler2DShadow, vec3, [float]) instead of
  // vec4 texture(sampler2D, vec2, [float]).
  // NVidia, AMD, and Intel all take this as a cue to provide "hardware PCF",
  // a driver hack that softens shadow edges with 4-sample interpolation.
  for (size_t i = 0; i < sceneMgr->getShadowTextureCount(); ++i)
  {
    const Ogre::TexturePtr tex = sceneMgr->getShadowTexture(i);
    // This will fail if not using OpenGL as the rendering backend.
    GLuint texId;
    tex->getCustomAttribute("GLID", &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
        GL_COMPARE_R_TO_TEXTURE);
  }
#endif

  sceneMgr->setShadowTextureSelfShadow(false);
  sceneMgr->setShadowCasterRenderBackFaces(true);

  // TODO: We have two different shadow caster materials, both taken from
  // OGRE samples. They should be compared and tested.
  // Set up caster material - this is just a standard depth/shadow map caster
  // sceneMgr->setShadowTextureCasterMaterial("PSSM/shadow_caster");
  sceneMgr->setShadowTextureCasterMaterial("Gazebo/shadow_caster");

  // Disable fog on the caster pass.
  //  Ogre::MaterialPtr passCaterMaterial =
  //   Ogre::MaterialManager::getSingleton().getByName("PSSM/shadow_caster");
  // Ogre::Pass* pssmCasterPass =
  // passCaterMaterial->getTechnique(0)->getPass(0);
  // pssmCasterPass->setFog(true);

  // shadow camera setup
  if (this->dataPtr->pssmSetup.isNull())
  {
    this->dataPtr->pssmSetup =
        Ogre::ShadowCameraSetupPtr(new CustomPSSMShadowCameraSetup());
  }

  sceneMgr->setShadowFarDistance(this->dataPtr->shadowFar);

  CustomPSSMShadowCameraSetup *cameraSetup =
      dynamic_cast<CustomPSSMShadowCameraSetup*>(
      this->dataPtr->pssmSetup.get());

  cameraSetup->calculateSplitPoints(3, this->dataPtr->shadowNear,
    this->dataPtr->shadowFar, this->dataPtr->shadowSplitLambda);
  cameraSetup->setSplitPadding(this->dataPtr->shadowSplitPadding);

  sceneMgr->setShadowCameraSetup(this->dataPtr->pssmSetup);

  this->dataPtr->shadowRenderState =
      this->dataPtr->shaderGenerator->createSubRenderState(
      CustomPSSM3::Type);
  CustomPSSM3 *pssm3SubRenderState =
      static_cast<CustomPSSM3 *>(this->dataPtr->shadowRenderState);

  const Ogre::PSSMShadowCameraSetup::SplitPointList &srcSplitPoints =
    cameraSetup->getSplitPoints();

  CustomPSSM3::SplitPointList dstSplitPoints;

  for (unsigned int i = 0; i < srcSplitPoints.size(); ++i)
  {
    dstSplitPoints.push_back(srcSplitPoints[i]);
  }

  pssm3SubRenderState->setSplitPoints(dstSplitPoints);
  schemRenderState->addTemplateSubRenderState(this->dataPtr->shadowRenderState);

  this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  this->UpdateShaders();

  this->dataPtr->shadowsApplied = true;
}

/////////////////////////////////////////////////
void RTShaderSystem::ReapplyShadows()
{
  if (this->dataPtr->shadowsApplied)
  {
    for (unsigned int i = 0; i < this->dataPtr->scenes.size(); i++)
    {
      RemoveShadows(this->dataPtr->scenes[i]);
      ApplyShadows(this->dataPtr->scenes[i]);
    }
  }
}

/////////////////////////////////////////////////
Ogre::PSSMShadowCameraSetup *RTShaderSystem::GetPSSMShadowCameraSetup() const
{
  return dynamic_cast<Ogre::PSSMShadowCameraSetup *>(
      this->dataPtr->pssmSetup.get());
}

/////////////////////////////////////////////////
void RTShaderSystem::Update()
{
  if (!this->dataPtr->initialized || !this->dataPtr->updateShaders)
    return;

  for (const auto &scene : this->dataPtr->scenes)
  {
    VisualPtr vis = scene->WorldVisual();
    if (vis)
    {
      this->UpdateShaders(vis);
    }
  }
  this->dataPtr->updateShaders = false;
}

/////////////////////////////////////////////////
bool RTShaderSystem::SetShadowTextureSize(const unsigned int _size)
{
  // check if texture size is a power of 2
  if (!ignition::math::isPowerOfTwo(_size))
  {
    gzerr << "Shadow texture size must be a power of 2" << std::endl;
    return false;
  }

  this->dataPtr->shadowTextureSize = _size;
  return true;
}

/////////////////////////////////////////////////
unsigned int RTShaderSystem::ShadowTextureSize() const
{
  return this->dataPtr->shadowTextureSize;
}

/////////////////////////////////////////////////
void RTShaderSystem::SetShadowClipDist(const double _near, const double _far)
{
  this->dataPtr->shadowNear = _near;
  this->dataPtr->shadowFar = _far;
  ReapplyShadows();
}

/////////////////////////////////////////////////
double RTShaderSystem::ShadowNearClip() const
{
  return this->dataPtr->shadowNear;
}

/////////////////////////////////////////////////
double RTShaderSystem::ShadowFarClip() const
{
  return this->dataPtr->shadowFar;
}

/////////////////////////////////////////////////
void RTShaderSystem::SetShadowSplitLambda(const double _lambda)
{
  this->dataPtr->shadowSplitLambda = _lambda;
  ReapplyShadows();
}

/////////////////////////////////////////////////
double RTShaderSystem::ShadowSplitLambda() const
{
  return this->dataPtr->shadowSplitLambda;
}

/////////////////////////////////////////////////
void RTShaderSystem::SetShadowSplitPadding(const double _padding)
{
  this->dataPtr->shadowSplitPadding = _padding;
  ReapplyShadows();
}

/////////////////////////////////////////////////
double RTShaderSystem::ShadowSplitPadding() const
{
  return this->dataPtr->shadowSplitPadding;
}
