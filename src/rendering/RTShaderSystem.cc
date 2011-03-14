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
 * Date: 27 Jan 2010
 * SVN: $Id:$
 */

#include <boost/bind.hpp>
#include <sys/stat.h>
#include <iostream>

#include "Scene.hh"
#include "Visual.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "RTShaderSystem.hh"
#include "gz.h"

#define MINOR_VERSION 700
using namespace gazebo;
using namespace rendering;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
RTShaderSystem::RTShaderSystem()
{
  this->initialized = false;
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

    /*for (unsigned int s=0; s < OgreAdaptor::Instance()->GetSceneMgrCount();s++)
    {
      std::string sceneName = OgreAdaptor::Instance()->GetSceneMgr(s)->getName() + Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME;
      std::cout << "SceneName[" << sceneName << "]\n";

      // Set the scene manager
      this->shaderGenerator->addSceneManager(
          OgreAdaptor::Instance()->GetSceneMgr(s));
    }*/

    this->initialized = true;
  }
  else
    gzerr(0) << "RT Shader system failed to initialize\n";
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
void RTShaderSystem::AddScene(Scene *scene)
{
#if INCLUDE_RTSHADER && OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= MINOR_VERSION
  // Set the scene manager
  this->shaderGenerator->addSceneManager( scene->GetManager() );
  this->shaderGenerator->createScheme( scene->GetName() +Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME  );
  this->scenes.push_back(scene);
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Remove a scene
void RTShaderSystem::RemoveScene( Scene *scene )
{
  std::vector<Scene*>::iterator iter;
  for (iter = this->scenes.begin(); iter != scenes.end(); iter++)
    if ( (*iter) == scene )
      break;

  if (iter != this->scenes.end())
  {
    delete *iter;
    this->scenes.erase(iter);
  }
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
void RTShaderSystem::AttachViewport(Ogre::Viewport *viewport, Scene *scene)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  viewport->setMaterialScheme( scene->GetName() +
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
  if (!this->initialized || !vis->GetUseRTShader())
    return;

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
            this->scenes[s]->GetIdString() + 
            Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);


        // Setup custom shader sub render states according to current setup.
        if (success)
        {
          Ogre::MaterialPtr curMaterial = 
            Ogre::MaterialManager::getSingleton().getByName(curMaterialName);

          Ogre::Pass* curPass = curMaterial->getTechnique(0)->getPass(0);

          // Grab the first pass render state. 
          // NOTE:For more complicated samples iterate over the passes and build
          // each one of them as desired.
          Ogre::RTShader::RenderState* renderState = 
            this->shaderGenerator->getRenderState(
                this->scenes[s]->GetIdString() + 
                Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME, 
                curMaterialName, 0);

          // Remove all sub render states.
          renderState->reset();

#ifdef RTSHADER_SYSTEM_BUILD_CORE_SHADERS
          if (vis->GetShader() == "vertex")
          {
            Ogre::RTShader::SubRenderState* perPerVertexLightModel = this->shaderGenerator->createSubRenderState(Ogre::RTShader::FFPLighting::Type);

            renderState->addTemplateSubRenderState(perPerVertexLightModel);
          }
#endif

#ifdef RTSHADER_SYSTEM_BUILD_EXT_SHADERS
          else if (vis->GetShader() == "pixel")
          {
            Ogre::RTShader::SubRenderState* perPixelLightModel = this->shaderGenerator->createSubRenderState(Ogre::RTShader::PerPixelLighting::Type);

            renderState->addTemplateSubRenderState(perPixelLightModel);
          }

          else if (vis->GetShader() == "normal_map_objectspace")
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
          }
#endif
        }


      // Invalidate this material in order to re-generate its shaders.
      this->shaderGenerator->invalidateMaterial(
          this->scenes[s]->GetIdString() + 
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
          struct stat astat;
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
    gzerr(0) << "Unable to find shader lib. Shader generating will fail.";
    return false;
  }

  return true;
}


void RTShaderSystem::ApplyShadows(Scene *scene)
{
  Ogre::SceneManager *sceneMgr;
  Ogre::RTShader::RenderState* schemeRenderState;

  sceneMgr = this->shaderGenerator->getActiveSceneManager();

	schemeRenderState = this->shaderGenerator->getRenderState(
      scene->GetIdString() + 
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE_INTEGRATED);
  sceneMgr->setShadowFarDistance(1000);
  sceneMgr->setShadowTexturePixelFormat(Ogre::PF_FLOAT16_R);
  sceneMgr->setShadowColour(Ogre::ColourValue(0.5, 0.5, 0.5));


  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);

  // 3 textures per directional light
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_POINT, 3);
  sceneMgr->setShadowTextureSettings(1024, 3, Ogre::PF_FLOAT16_R);
  sceneMgr->setShadowTextureSelfShadow(true);

  // Disable fog on the caster pass.
  /*Ogre::MaterialPtr passCaterMaterial
    //= Ogre::MaterialManager::getSingleton().getByName("PSSM/shadow_caster");
    = Ogre::MaterialManager::getSingleton().getByName("pssm_vsm_caster");
  Ogre::Pass* pssmCasterPass
    = passCaterMaterial->getTechnique(0)->getPass(0);
  pssmCasterPass->setFog(true);
*/
    
  // shadow camera setup
  Ogre::PSSMShadowCameraSetup* pssmSetup = new Ogre::PSSMShadowCameraSetup();
  pssmSetup->calculateSplitPoints(3, 0.1, 1000);
  pssmSetup->setSplitPadding(.5);
  pssmSetup->setUseSimpleOptimalAdjust(true);
  pssmSetup->setOptimalAdjustFactor(0, 3);
  pssmSetup->setOptimalAdjustFactor(1, 1);
  pssmSetup->setOptimalAdjustFactor(2, 0.5);

  sceneMgr->setShadowCameraSetup(Ogre::ShadowCameraSetupPtr(pssmSetup));

  // Set up caster material - this is just a standard depth/shadow map caster
  //sceneMgr->setShadowTextureCasterMaterial("PSSM/shadow_caster");
  sceneMgr->setShadowTextureCasterMaterial("pssm_vsm_caster");
  sceneMgr->setShadowCasterRenderBackFaces(false);

  Ogre::RTShader::SubRenderState *subRenderState;
  Ogre::RTShader::IntegratedPSSM3 *pssm3SubRenderState;
  Ogre::RTShader::IntegratedPSSM3::SplitPointList dstSplitPoints;

  subRenderState = this->shaderGenerator->createSubRenderState(Ogre::RTShader::IntegratedPSSM3::Type);	
  pssm3SubRenderState = static_cast<Ogre::RTShader::IntegratedPSSM3*>(subRenderState);
  const Ogre::PSSMShadowCameraSetup::SplitPointList& srcSplitPoints = pssmSetup->getSplitPoints();

  for (unsigned int i=0; i < srcSplitPoints.size(); ++i)
  {
    dstSplitPoints.push_back(srcSplitPoints[i]);
  }

  pssm3SubRenderState->setSplitPoints(dstSplitPoints);
  schemeRenderState->addTemplateSubRenderState(subRenderState);		

  // set the receiver params for any materials that need the split point information
  Ogre::Vector4 splitPoints;
  for (int i = 0; i < srcSplitPoints.size(); ++i)
  {
    splitPoints[i] = srcSplitPoints[i];
  }

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName("Gazebo/GrayGrid");
  for(int i = 0; i < mat->getNumTechniques(); ++i) 
  {
    mat->getTechnique(i)->getPass(1)->getFragmentProgramParameters()->setNamedConstant("pssmSplitPoints", splitPoints);
  }

  this->shaderGenerator->invalidateScheme(
      scene->GetIdString() + 
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
}
