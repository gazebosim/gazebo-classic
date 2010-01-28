/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Wrapper around the OGRE RTShader system
 * Author: Nate Koenig
 * Date: 27 Jan 2010
 * SVN: $Id:$
 */

#include <boost/bind.hpp>
#include "World.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "OgreAdaptor.hh"
#include "RTShaderSystem.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
RTShaderSystem::RTShaderSystem()
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  World::Instance()->ConnectPerPixelLightingSignal( boost::bind(&RTShaderSystem::SetPerPixelLighting, this, _1) );
  this->curLightingModel = RTShaderSystem::SSLM_PerPixelLighting;
#endif
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
RTShaderSystem::~RTShaderSystem()
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  // Restore default scheme.
  Ogre::MaterialManager::getSingleton().setActiveScheme(Ogre::MaterialManager::DEFAULT_SCHEME_NAME);

  // Unregister the material manager listener.
  if (this->materialMgrListener != NULL)
  {
    Ogre::MaterialManager::getSingleton().removeListener(
        this->materialMgrListener);
    delete this->materialMgrListener;
    this->materialMgrListener = NULL;
  }

  // Finalize RTShader system.
  if (this->shaderGenerator != NULL)
  {
    Ogre::RTShader::ShaderGenerator::finalize();
    this->shaderGenerator = NULL;
  }
#endif
}

////////////////////////////////////////////////////////////////////////////////
/// Init the run time shader system
void RTShaderSystem::Init()
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  if (Ogre::RTShader::ShaderGenerator::initialize())
  {
    Ogre::StringVector groupVector;

    this->shaderGenerator = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
    this->shaderGenerator->addSceneManager(OgreAdaptor::Instance()->sceneMgr);
 
    // Setup the core libraries and shader cache path 
    groupVector =Ogre::ResourceGroupManager::getSingleton().getResourceGroups();
    Ogre::StringVector::iterator itGroup = groupVector.begin();
    Ogre::StringVector::iterator itGroupEnd = groupVector.end();
    Ogre::String shaderCoreLibsPath;
    Ogre::String shaderCachePath;

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
        if ((*it)->archive->getName().find("rtshaderlib") != Ogre::String::npos)
        {
          shaderCoreLibsPath = (*it)->archive->getName() + "/";
          shaderCachePath = shaderCoreLibsPath;
          coreLibsFound = true;
          break;
        }
      }

      // Core libs path found in the current group.
      if (coreLibsFound)
        break;
    }

    // Core shader lib not found -> shader generating will fail.
    if (shaderCoreLibsPath.empty())
    {
      gzerr(0) << "Unable to find shader lib. Shader generating will fail.";
      return;
    }

#ifdef _RTSS_WRITE_SHADERS_TO_DISK
    // Set shader cache path.
   this->shaderGenerator->setShaderCachePath(shaderCachePath);
#endif

    // Create and register the material manager listener.  
    this->materialMgrListener = new ShaderGeneratorTechniqueResolverListener(this->shaderGenerator);
    Ogre::MaterialManager::getSingleton().addListener(this->materialMgrListener);

    this->shaderGenerator->setTargetLanguage("glsl");

    // Grab the scheme render state.                        
    Ogre::RTShader::RenderState* schemRenderState = this->shaderGenerator->getRenderState(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

    Ogre::RTShader::SubRenderState* perPixelLightModel = this->shaderGenerator->createSubRenderState(Ogre::RTShader::PerPixelLighting::Type);

    schemRenderState->addSubRenderState(perPixelLightModel);

    // Invalidate the scheme in order to re-generate all shaders based technique related to this scheme.
    this->shaderGenerator->invalidateScheme(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
  }
  else
    gzerr(0) << "RT Shader system failed to initialize\n";
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Set an Ogre::Entity to use RT shaders
void RTShaderSystem::AttachEntity(Ogre::Entity *entity)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  this->GenerateShaders(entity);
  this->entities.push_back(entity);
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Remove and entity
void RTShaderSystem::DetachEntity(Ogre::Entity *entity)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  this->entities.remove(entity);
#endif
}

////////////////////////////////////////////////////////////////////////////////
/// Set the lighting model
void RTShaderSystem::SetLightingModel(LightingModel model)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  if (model == this->curLightingModel)
    return;

  this->curLightingModel = model;

  std::list<Ogre::Entity*>::iterator iter;

  // Update all the shaders
  for (iter = this->entities.begin(); iter != this->entities.end(); iter++)
  {
    this->GenerateShaders(*iter);
  }
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Set the lighting model to per pixel or per vertex
void RTShaderSystem::SetPerPixelLighting( bool s)
{
  if (s)
    this->SetLightingModel( SSLM_PerPixelLighting );
  else
    this->SetLightingModel( SSLM_PerVertexLighting );
}

////////////////////////////////////////////////////////////////////////////////
/// Generate shaders for an entity
void RTShaderSystem::GenerateShaders(Ogre::Entity *entity)
{
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7

  for (unsigned int i=0; i < entity->getNumSubEntities(); ++i)
  {
    Ogre::SubEntity* curSubEntity = entity->getSubEntity(i);
    const Ogre::String& curMaterialName = curSubEntity->getMaterialName();
    bool success;

    // Create the shader based technique of this material.
    success = this->shaderGenerator->createShaderBasedTechnique(curMaterialName,
        Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

    // Setup custom shader sub render states according to current setup.
    if (success)
    {
      Ogre::MaterialPtr curMaterial = 
        Ogre::MaterialManager::getSingleton().getByName(curMaterialName);

      Ogre::Pass* curPass = curMaterial->getTechnique(0)->getPass(0);

      /*if (mSpecularEnable)
      {
        curPass->setSpecular(ColourValue::White);
        curPass->setShininess(32.0);
      }
      else
      {
        curPass->setSpecular(ColourValue::Black);
        curPass->setShininess(0.0);
      }*/

      // Grab the first pass render state. 
      // NOTE: For more complicated samples iterate over the passes and build
      // each one of them as desired.
      Ogre::RTShader::RenderState* renderState = this->shaderGenerator->getRenderState(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME, curMaterialName, 0);

      // Remove all sub render states.
      renderState->reset();

#ifdef RTSHADER_SYSTEM_BUILD_CORE_SHADERS
      if (this->curLightingModel == SSLM_PerVertexLighting)
      {
        Ogre::RTShader::SubRenderState* perPerVertexLightModel = this->shaderGenerator->createSubRenderState(Ogre::RTShader::FFPLighting::Type);

        renderState->addSubRenderState(perPerVertexLightModel);
      }
#endif

#ifdef RTSHADER_SYSTEM_BUILD_EXT_SHADERS
      else if (this->curLightingModel == SSLM_PerPixelLighting)
      {
        Ogre::RTShader::SubRenderState* perPixelLightModel = this->shaderGenerator->createSubRenderState(Ogre::RTShader::PerPixelLighting::Type);

        renderState->addSubRenderState(perPixelLightModel);
      }
    }

    // Invalidate this material in order to re-generate its shaders.
    this->shaderGenerator->invalidateMaterial(Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME, curMaterialName);


  }
#endif


#endif
}


