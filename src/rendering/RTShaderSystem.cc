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
/* Desc: Wrapper around the OGRE RTShader system
 * Author: Nate Koenig
 */

#include <boost/bind.hpp>
#include <sys/stat.h>

#include "rendering/Scene.hh"
#include "rendering/Visual.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "rendering/RTShaderSystem.hh"

#define MINOR_VERSION 7
using namespace gazebo;
using namespace rendering;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
RTShaderSystem::RTShaderSystem()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
RTShaderSystem::~RTShaderSystem()
{
  this->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Init the run time shader system
void RTShaderSystem::Init()
{
#if INCLUDE_RTSHADER && OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= MINOR_VERSION
  if (Ogre::RTShader::ShaderGenerator::initialize())
  {
    std::string coreLibsPath, cachePath;
    this->GetPaths(coreLibsPath, cachePath);

    // Get the shader generator pointer
    this->shaderGenerator = Ogre::RTShader::ShaderGenerator::getSingletonPtr();

    // Add the shader libs resource location
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        coreLibsPath, "FileSystem");

    // Set shader cache path.
    this->shaderGenerator->setShaderCachePath(cachePath);

    this->shaderGenerator->setTargetLanguage("glsl");

    this->initialized = true;
  }
  else
    gzerr << "RT Shader system failed to initialize\n";

#endif
}

////////////////////////////////////////////////////////////////////////////////
// Finalize
void RTShaderSystem::Fini()
{
  if (!this->initialized)
    return;

  // Restore default scheme.
  Ogre::MaterialManager::getSingleton().setActiveScheme(Ogre::MaterialManager::DEFAULT_SCHEME_NAME);

  // Finalize RTShader system.
  if (this->shaderGenerator != NULL)
  {
    Ogre::RTShader::ShaderGenerator::finalize();
    this->shaderGenerator = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Add a scene manager
void RTShaderSystem::AddScene(Scene *_scene)
{
#if INCLUDE_RTSHADER && OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= MINOR_VERSION
  // Set the scene manager
  this->shaderGenerator->addSceneManager( _scene->GetManager() );
  this->shaderGenerator->createScheme( _scene->GetName() +Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME  );
  this->scenes.push_back(_scene);
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Remove a scene
void RTShaderSystem::RemoveScene( Scene *_scene )
{
  //std::remove(this->scenes.begin(), this->scenes.end(), scene);

  std::cout << "Remove Scene[" << this->scenes.size() << "]\n";
  std::vector<Scene*>::iterator iter;
  for (iter = this->scenes.begin(); iter != scenes.end(); iter++)
    if ( (*iter) == _scene )
      break;

  if (iter != this->scenes.end())
  {
    this->scenes.erase(iter);
    this->shaderGenerator->invalidateScheme( _scene->GetName() + Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME );
    this->UpdateShaders();
  }

  std::cout << "   Remove Scene[" << this->scenes.size() << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
// Set an Ogre::Entity to use RT shaders
void RTShaderSystem::AttachEntity(Visual *vis)
{
  if (!this->initialized)
    return;

  this->entities.push_back(vis);
  this->GenerateShaders(vis);
}

////////////////////////////////////////////////////////////////////////////////
// Remove and entity
void RTShaderSystem::DetachEntity(Visual *vis)
{
  if (!this->initialized)
    return;

  this->entities.remove(vis);
}

////////////////////////////////////////////////////////////////////////////////
/// Set a viewport to use shaders
void RTShaderSystem::AttachViewport(Ogre::Viewport *_viewport, Scene *_scene)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  _viewport->setMaterialScheme( _scene->GetName() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

#endif
}


////////////////////////////////////////////////////////////////////////////////
/// Update the shaders
void RTShaderSystem::UpdateShaders()
{
  if (!this->initialized)
    return;


  std::list<Visual*>::iterator iter;

  // Update all the shaders
  for (iter = this->entities.begin(); iter != this->entities.end(); iter++)
    this->GenerateShaders(*iter);
}

////////////////////////////////////////////////////////////////////////////////
/// Generate shaders for an entity
void RTShaderSystem::GenerateShaders(Visual *vis)
{
  if (!this->initialized)
  {
    gzwarn << "RTShader system not initialized.\n";
    return;
  }

  for (unsigned int k=0; k < vis->GetSceneNode()->numAttachedObjects(); k++)
  {
    Ogre::MovableObject *obj = vis->GetSceneNode()->getAttachedObject(k);
    Ogre::Entity *entity = dynamic_cast<Ogre::Entity*>(obj);
    if (!entity)
      continue;

    for (unsigned int i=0; i < entity->getNumSubEntities(); ++i)
    {
      Ogre::SubEntity* curSubEntity = entity->getSubEntity(i);
      const Ogre::String& curMaterialName = curSubEntity->getMaterialName();
      bool success;

      for (unsigned int s=0; s < this->scenes.size(); s++)
      {

        // Create the shader based technique of this material.
        success = this->shaderGenerator->createShaderBasedTechnique(
            curMaterialName,
            Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
            this->scenes[s]->GetName() + 
            Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);


        // Setup custom shader sub render states according to current setup.
        if (success)
        {
          Ogre::MaterialPtr curMaterial = 
            Ogre::MaterialManager::getSingleton().getByName(curMaterialName);

          // Grab the first pass render state. 
          // NOTE:For more complicated samples iterate over the passes and build
          // each one of them as desired.
          Ogre::RTShader::RenderState* renderState = 
            this->shaderGenerator->getRenderState(
                this->scenes[s]->GetName() + 
                Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME, 
                curMaterialName, 0);

          // Remove all sub render states.
          renderState->reset();
/*
#ifdef RTSHADER_SYSTEM_BUILD_CORE_SHADERS
          if (vis->GetShader() == "vertex")
          {
            Ogre::RTShader::SubRenderState* perPerVertexLightModel = this->shaderGenerator->createSubRenderState(Ogre::RTShader::FFPLighting::Type);

            renderState->addTemplateSubRenderState(perPerVertexLightModel);
          }
#endif
*/

#ifdef RTSHADER_SYSTEM_BUILD_EXT_SHADERS
          //else if (vis->GetShader() == "pixel")
          {
            Ogre::RTShader::SubRenderState* perPixelLightModel = this->shaderGenerator->createSubRenderState(Ogre::RTShader::PerPixelLighting::Type);

            renderState->addTemplateSubRenderState(perPixelLightModel);
          }

          /*else if (vis->GetShader() == "normal_map_objectspace")
          {
            Ogre::RTShader::SubRenderState* subRenderState = this->shaderGenerator->createSubRenderState(Ogre::RTShader::NormalMapLighting::Type);
            Ogre::RTShader::NormalMapLighting* normalMapSubRS = static_cast<Ogre::RTShader::NormalMapLighting*>(subRenderState);

            normalMapSubRS->setNormalMapSpace(Ogre::RTShader::NormalMapLighting::NMS_OBJECT);
            normalMapSubRS->setNormalMapTextureName(vis->GetNormalMap());
            renderState->addTemplateSubRenderState(normalMapSubRS);
          }
          else if (vis->GetShader() == "normal_map_tangetspace")
          {
            Ogre::RTShader::SubRenderState* subRenderState = this->shaderGenerator->createSubRenderState(Ogre::RTShader::NormalMapLighting::Type);
            Ogre::RTShader::NormalMapLighting* normalMapSubRS = static_cast<Ogre::RTShader::NormalMapLighting*>(subRenderState);

            normalMapSubRS->setNormalMapSpace(Ogre::RTShader::NormalMapLighting::NMS_TANGENT);
            normalMapSubRS->setNormalMapTextureName(vis->GetNormalMap());

            renderState->addTemplateSubRenderState(normalMapSubRS);
          }*/
#endif
        }


      // Invalidate this material in order to re-generate its shaders.
      this->shaderGenerator->invalidateMaterial(
          this->scenes[s]->GetName() + 
          Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME, curMaterialName); 
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get paths for the shader system
bool RTShaderSystem::GetPaths(std::string &coreLibsPath, std::string &cachePath)
{
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

    resLocationsList = Ogre::ResourceGroupManager::getSingleton().getResourceLocationList(*itGroup);
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
            tmpdir = (char*)"/tmp";
          // Get the user
          user = getenv("USER");
          if (!user)
            user = (char*)"nobody";
          stream << tmpdir << "/gazebo-" << user << "-rtshaderlibcache" << "/";
          cachePath = stream.str();
          // Create the directory
          if (mkdir(cachePath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR) != 0)
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

void RTShaderSystem::RemoveShadows(Scene *_scene)
{
  _scene->GetManager()->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  _scene->GetManager()->setShadowCameraSetup(Ogre::ShadowCameraSetupPtr());

  Ogre::RTShader::RenderState* schemeRenderState = 
    this->shaderGenerator->getRenderState(
        _scene->GetName() + 
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  schemeRenderState->removeTemplateSubRenderState(this->shadowRenderState);

  this->shaderGenerator->invalidateScheme(_scene->GetName() + 
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
  this->UpdateShaders();
}

void RTShaderSystem::ApplyShadows(Scene *scene)
{
  Ogre::SceneManager *sceneMgr = scene->GetManager();
  //sceneMgr = this->shaderGenerator->getActiveSceneManager();

  // Grab the scheme render state.												
  Ogre::RTShader::RenderState* schemRenderState = this->shaderGenerator->getRenderState(scene->GetName() + Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE_INTEGRATED);

  // 3 textures per directional light
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_DIRECTIONAL, 3);
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_POINT, 0);
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_SPOTLIGHT, 0);
  sceneMgr->setShadowTextureSettings(1024, 3, Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureSelfShadow(true);

  // Set up caster material - this is just a standard depth/shadow map caster
  sceneMgr->setShadowTextureCasterMaterial("PSSM/shadow_caster");


  // Disable fog on the caster pass.
  Ogre::MaterialPtr passCaterMaterial = Ogre::MaterialManager::getSingleton().getByName("PSSM/shadow_caster");
  Ogre::Pass* pssmCasterPass = passCaterMaterial->getTechnique(0)->getPass(0);
  pssmCasterPass->setFog(true);

  // shadow camera setup
  Ogre::PSSMShadowCameraSetup* pssmSetup = new Ogre::PSSMShadowCameraSetup();
  pssmSetup->calculateSplitPoints(3, 1, 1000);
  pssmSetup->setSplitPadding(10);
  pssmSetup->setOptimalAdjustFactor(0, 2);
  pssmSetup->setOptimalAdjustFactor(1, 1);
  pssmSetup->setOptimalAdjustFactor(2, 0.5);

  sceneMgr->setShadowCameraSetup(Ogre::ShadowCameraSetupPtr(pssmSetup));

  this->shadowRenderState = this->shaderGenerator->createSubRenderState(Ogre::RTShader::IntegratedPSSM3::Type);	
  Ogre::RTShader::IntegratedPSSM3* pssm3SubRenderState = static_cast<Ogre::RTShader::IntegratedPSSM3*>(this->shadowRenderState);
  const Ogre::PSSMShadowCameraSetup::SplitPointList& srcSplitPoints = pssmSetup->getSplitPoints();
  Ogre::RTShader::IntegratedPSSM3::SplitPointList dstSplitPoints;

  for (unsigned int i=0; i < srcSplitPoints.size(); ++i)
  {
    dstSplitPoints.push_back(srcSplitPoints[i]);
  }

  pssm3SubRenderState->setSplitPoints(dstSplitPoints);
  schemRenderState->addTemplateSubRenderState(this->shadowRenderState);		


  this->shaderGenerator->invalidateScheme(scene->GetName() + Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  this->UpdateShaders();
}
