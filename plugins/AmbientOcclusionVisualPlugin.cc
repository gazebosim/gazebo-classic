/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <mutex>

#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/ogre_gazebo.h>

#include "AmbientOcclusionVisualPlugin.hh"

namespace gazebo
{
  class SSAOGBufferSchemeHandler : public Ogre::MaterialManager::Listener
  {
    public:
      SSAOGBufferSchemeHandler()
      {
        mGBufRefMat = Ogre::MaterialManager::getSingleton().getByName("SSAO/GBuffer");
      }

      virtual ~SSAOGBufferSchemeHandler()
      {
        mGBufRefMat.setNull();
      }

      /** @copydoc MaterialManager::Listener::handleSchemeNotFound */
      virtual Ogre::Technique* handleSchemeNotFound(unsigned short /*schemeIndex*/,
        const Ogre::String& schemeName, Ogre::Material *originalMaterial, unsigned short /*lodIndex*/,
        const Ogre::Renderable * /*rend*/)
      {
          Ogre::Technique *gBufferTech = originalMaterial->createTechnique();
          gBufferTech->setSchemeName(schemeName);
          Ogre::Pass* gbufPass = gBufferTech->createPass();
          *gbufPass = *mGBufRefMat->getTechnique(0)->getPass(0);
          return gBufferTech;
      }

    private:
      Ogre::MaterialPtr mGBufRefMat;
  };

  /// \brief Private data for the AmbientOcclusionVisualPlugin class.
  class AmbientOcclusionVisualPluginPrivate
  {
    public: std::string compositorName;
    public: std::string postFilterName;
    public: SSAOGBufferSchemeHandler *GBufSchemeHandler = nullptr;

    public: void AddSSAO(rendering::CameraPtr _cam);
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(AmbientOcclusionVisualPlugin)

/////////////////////////////////////////////////
AmbientOcclusionVisualPlugin::AmbientOcclusionVisualPlugin()
    : dataPtr(new AmbientOcclusionVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
AmbientOcclusionVisualPlugin::~AmbientOcclusionVisualPlugin()
{
  Ogre::MaterialManager::getSingleton().removeListener(
      this->dataPtr->GBufSchemeHandler, "GBuffer");
  delete this->dataPtr->GBufSchemeHandler;
  this->dataPtr->GBufSchemeHandler = nullptr;
}

/////////////////////////////////////////////////
void AmbientOcclusionVisualPlugin::Load(rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }
//  this->dataPtr->visual = _visual;


  this->dataPtr->compositorName = "SSAO/CreaseShading";
  this->dataPtr->postFilterName = "SSAO/Post/NoFilter";

  rendering::ScenePtr scene = _visual->GetScene();
  if (!scene)
  {
    gzerr << "Scene is null. Ambient Occlusion will not be enabled" << std::endl;
    return;
  }

  for (unsigned int i = 0; i < scene->CameraCount(); ++i)
  {
    this->dataPtr->AddSSAO(scene->GetCamera(i));
  }
  for (unsigned int i = 0; i < scene->UserCameraCount(); ++i)
  {
    rendering::CameraPtr cam =
        boost::dynamic_pointer_cast<rendering::Camera>(
        scene->GetUserCamera(i));
    this->dataPtr->AddSSAO(cam);
  }

  this->dataPtr->GBufSchemeHandler = new SSAOGBufferSchemeHandler();
  Ogre::MaterialManager::getSingleton().addListener(
      this->dataPtr->GBufSchemeHandler, "GBuffer");
}

/////////////////////////////////////////////////
void AmbientOcclusionVisualPluginPrivate::AddSSAO(rendering::CameraPtr _cam)
{
  Ogre::Viewport *ogreViewport = _cam->OgreCamera()->getViewport();
  if (!ogreViewport)
  {
    gzerr << "Viewport is null. Ambient Occlusion will not be enabled"
          << std::endl;
    return;
  }

  // GBuffer
  if (Ogre::CompositorManager::getSingleton().addCompositor(
      ogreViewport, "SSAO/GBuffer"))
  {
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(
        ogreViewport, "SSAO/GBuffer", true);
  }
  else
  {
    gzerr << "Failed to add GBuffer compositor" << std::endl;
  }

  // SSAO - Crease Shading
  if (Ogre::CompositorManager::getSingleton().addCompositor(
      ogreViewport, this->compositorName))
  {
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(
        ogreViewport, compositorName, true);
  }
  else
  {
      gzerr << "Failed to add compositor: " << this->compositorName
            << std::endl;
  }

  // SSAO post filter
  if (Ogre::CompositorManager::getSingleton().addCompositor(
      ogreViewport, this->postFilterName))
  {
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(
        ogreViewport, this->postFilterName, true);
  }
  else
  {
    gzerr << "Failed to add " << this->postFilterName << " compositor"
          << std::endl;
  }

  // modulate with existing scene
  std::string postModulateFilterName = "SSAO/Post/Modulate";
  if (Ogre::CompositorManager::getSingleton().addCompositor(
      ogreViewport, postModulateFilterName))
  {
    Ogre::CompositorManager::getSingleton().setCompositorEnabled(
        ogreViewport, postModulateFilterName, true);
  }
  else
  {
    gzerr << "Failed to add " << postModulateFilterName << " compositor"
          << std::endl;
  }
}
