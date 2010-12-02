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
#include "gazebo_config.h"

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
#include <RTShaderSystem/OgreRTShaderSystem.h>
#endif

#include "OgreCamera.hh"
#include "SingletonT.hh"

namespace gazebo
{
  class OgreVisual;
  class Scene;

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

    /// \brief Finalize the shader system
    public: void Fini();

    /// \brief Add a scene manager
    public: void AddScene(Scene *scene);

    /// \brief Update the shaders
    public: void UpdateShaders();

    /// \brief Set an Ogre::Entity to use RT shaders
    public: void AttachEntity(OgreVisual *vis);

    /// \brief Remove and entity
    public: void DetachEntity(OgreVisual *vis);

    /// \brief Set a viewport to use shaders
    public: static void AttachViewport(OgreCamera *camera);

    /// Set the lighting model to per pixel or per vertex
    public: void SetPerPixelLighting( bool s);

    /// \brief Generate shaders for an entity
    public: void GenerateShaders(OgreVisual *vis);

    /// \brief Get paths for the shader system
    //private: bool GetPaths(std::string &coreLibsPath, std::string &cachePath);

    public: void ApplyShadows();

    /// \brief Get paths for the shader system
    private: bool GetPaths(std::string &coreLibsPath, std::string &cachePath);

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 7
    private: Ogre::RTShader::ShaderGenerator *shaderGenerator;
    private: std::list<OgreVisual*> entities;
#endif

    private: bool initialized;

    private: friend class DestroyerT<RTShaderSystem>;
    private: friend class SingletonT<RTShaderSystem>;
  };

}

#endif
