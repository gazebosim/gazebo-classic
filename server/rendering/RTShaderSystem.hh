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

#ifndef RTSHADERSYSTEM_HH
#define RTSHADERSYSTEM_HH

#include <Ogre.h>
#include <list>
#include "config.h"

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
#include <RTShaderSystem/OgreRTShaderSystem.h>
#endif

#include "SingletonT.hh"

namespace gazebo
{
  class ShaderGeneratorTechniqueResolverListener;
  class OgreVisual;

  class RTShaderSystem : public SingletonT<RTShaderSystem>
  {
    public: enum LightingModel
            {
              SSLM_PerVertexLighting,
              SSLM_PerPixelLighting,
              SSLM_NormalMapLightingTangentSpace,
              SSLM_NormalMapLightingObjectSpace,
            };

    /// \brief Constructor
    private: RTShaderSystem();

    /// \brief Destructor
    private: virtual ~RTShaderSystem();

    /// \brief Init the run time shader system
    public: void Init();

    /// \brief Update the shaders
    public: void UpdateShaders();

    /// \brief Set an Ogre::Entity to use RT shaders
    public: void AttachEntity(OgreVisual *vis);

    /// \brief Remove and entity
    public: void DetachEntity(OgreVisual *vis);

    /// \brief Set a viewport to use shaders
    public: static void AttachViewport(Ogre::Viewport *vp)
            {
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
              vp->setMaterialScheme(
                  Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
#endif
            }

    /// Set the lighting model to per pixel or per vertex
    public: void SetPerPixelLighting( bool s);

    /// \brief Generate shaders for an entity
    private: void GenerateShaders(OgreVisual *vis);


#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
    private: Ogre::RTShader::ShaderGenerator *shaderGenerator;
    private: ShaderGeneratorTechniqueResolverListener *materialMgrListener;
    private: std::list<OgreVisual*> entities;
#endif

    private: friend class DestroyerT<RTShaderSystem>;
    private: friend class SingletonT<RTShaderSystem>;
  };


#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
  // This class demonstrates basic usage of the RTShader system.
  // It sub class the material manager listener class and when a target 
  // scheme callback is invoked with the shader generator scheme it tries to 
  // create an equivalent shader based technique based on the default 
  // technique of the given material.
  class ShaderGeneratorTechniqueResolverListener : public Ogre::MaterialManager::Listener
  {
    public: ShaderGeneratorTechniqueResolverListener(Ogre::RTShader::ShaderGenerator* pShaderGenerator)
            {
              mShaderGenerator = pShaderGenerator;
            }

            // This is the hook point where shader based technique will be created.
            // It will be called whenever the material manager won't find
            // appropriate technique that satisfy the target scheme name. 
            // If the scheme name is out target RT Shader System scheme name we will 
            // try to create shader generated technique for it. 
    public: virtual Ogre::Technique* handleSchemeNotFound(
                unsigned short schemeIndex, const Ogre::String& schemeName, 
                Ogre::Material* originalMaterial, unsigned short lodIndex, 
                const Ogre::Renderable* rend)
            {
              Ogre::Technique* generatedTech = NULL;

              // Case this is the default shader generator scheme.
              if (schemeName == 
                  Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME)
              {
                bool techniqueCreated;

                // Create shader generated technique for this material.
                techniqueCreated = mShaderGenerator->createShaderBasedTechnique(
                    originalMaterial->getName(), 
                    Ogre::MaterialManager::DEFAULT_SCHEME_NAME, 
                    schemeName);

                // Case technique registration succeeded.
                if (techniqueCreated)
                {
                  // Force creating the shaders for the generated technique.
                  mShaderGenerator->validateMaterial(schemeName, 
                                                   originalMaterial->getName());

                  // Grab the generated technique.
                  Ogre::Material::TechniqueIterator itTech = 
                                       originalMaterial->getTechniqueIterator();

                  while (itTech.hasMoreElements())
                  {
                    Ogre::Technique* curTech = itTech.getNext();

                    if (curTech->getSchemeName() == schemeName)
                    {
                      generatedTech = curTech;
                      break;
                    }
                  }
                }
              }

              return generatedTech;
            }

            // The shader generator instance.   
    protected: Ogre::RTShader::ShaderGenerator* mShaderGenerator;
  };
#endif
}

#endif
