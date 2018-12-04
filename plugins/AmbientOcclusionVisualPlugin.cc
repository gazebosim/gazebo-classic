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

#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/ogre_gazebo.h>

#include "AmbientOcclusionVisualPlugin.hh"

namespace gazebo
{
  /// \brief Helper class to assign the GBuffer material to compositors that
  /// need them
  class SsaoGBufferSchemeHandler : public Ogre::MaterialManager::Listener
  {
    /// \brief Constructor
    public: SsaoGBufferSchemeHandler()
    {
      this->gBufRefMat =
          Ogre::MaterialManager::getSingleton().getByName("SSAO/GBuffer");
      if (this->gBufRefMat.isNull())
      {
        gzerr << "Unable to find 'SSAO/GBuffer' material, SSAO will not work"
              << std::endl;
      }
    }

    /// \brief Destructor
    public: ~SsaoGBufferSchemeHandler()
    {
      this->gBufRefMat.setNull();
    }

    /// \brief Ogre callback for assigning the GBuffer material to compositors
    /// \param[in] _schemeIndex Index of scheme requested
    /// \param[in] _schemeName Name of scheme requested
    /// \param[in] _originalMaterial Orignal material that does not contain
    /// the requested scheme
    /// \param[in] _lodIndex The material level-of-detail
    /// \param[in] _rend Pointer to the Ogre::Renderable object requesting
    /// the use of the techinique
    /// \return The Ogre material technique to use when scheme is not found.
    public: virtual Ogre::Technique *handleSchemeNotFound(
        uint16_t /*_schemeIndex*/, const Ogre::String& _schemeName,
        Ogre::Material *_originalMaterial, uint16_t /*_lodIndex*/,
        const Ogre::Renderable * /*_rend*/)
    {
      // ignore transparent / semi-tranparent materials with alpha rejection set
      // This includes the clouds in skyx
      Ogre::Technique *origTech = _originalMaterial->getTechnique(0);
      if (origTech)
      {
        for (unsigned int i = 0; i < origTech->getNumPasses(); ++i)
        {
          Ogre::Pass *origPass = origTech->getPass(i);
          if (origPass && origPass->getAlphaRejectFunction() !=
              Ogre::CMPF_ALWAYS_PASS)
          {
            auto tech = _originalMaterial->createTechnique();
            tech->setSchemeName(_schemeName);
            tech->removeAllPasses();
            return tech;
          }
        }
      }

      // set to use gbuffer
      Ogre::Technique *gBufferTech = _originalMaterial->createTechnique();
      gBufferTech->setSchemeName(_schemeName);
      Ogre::Pass *gbufPass = gBufferTech->createPass();
      if (!this->gBufRefMat.isNull())
        *gbufPass = *this->gBufRefMat->getTechnique(0)->getPass(0);
      return gBufferTech;
    }

    /// \brief GBuffer material
    private: Ogre::MaterialPtr gBufRefMat;
  };

  /// \brief Private data for the AmbientOcclusionVisualPlugin class.
  class AmbientOcclusionVisualPluginPrivate
  {
    /// \brief Destructor
    public: ~AmbientOcclusionVisualPluginPrivate();

    /// \brief Ambient occlusion compositor name
    public: std::string compositorName;

    /// \brief Post filter compositor name
    public: std::string postFilterName;

    /// \brief GBuffer material scheme handler
    public: SsaoGBufferSchemeHandler *gBufSchemeHandler = nullptr;

    /// \brief Apply ambient occlusion to the viewport of the input camera
    /// \param[in] _cam Pointer to a camera
    public: void AddSsao(rendering::CameraPtr _cam);
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(AmbientOcclusionVisualPlugin)

/////////////////////////////////////////////////
AmbientOcclusionVisualPluginPrivate::~AmbientOcclusionVisualPluginPrivate()
{
  Ogre::MaterialManager::getSingleton().removeListener(
      this->gBufSchemeHandler, "GBuffer");
  delete this->gBufSchemeHandler;
  this->gBufSchemeHandler = nullptr;
}

/////////////////////////////////////////////////
AmbientOcclusionVisualPlugin::AmbientOcclusionVisualPlugin()
    : dataPtr(new AmbientOcclusionVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
AmbientOcclusionVisualPlugin::~AmbientOcclusionVisualPlugin()
{
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

  // Use Crease Shading. There are others but this one is fast
  // and gives reasonably nice looking results
  this->dataPtr->compositorName = "SSAO/CreaseShading";
  this->dataPtr->postFilterName = "SSAO/Post/NoFilter";

  rendering::ScenePtr scene = _visual->GetScene();
  if (!scene)
  {
    gzerr << "Scene is null. Ambient Occlusion will not be enabled"
          << std::endl;
    return;
  }

  // apply to all cameras
  for (unsigned int i = 0; i < scene->CameraCount(); ++i)
  {
    this->dataPtr->AddSsao(scene->GetCamera(i));
  }
  for (unsigned int i = 0; i < scene->UserCameraCount(); ++i)
  {
    rendering::CameraPtr cam =
        boost::dynamic_pointer_cast<rendering::Camera>(
        scene->GetUserCamera(i));
    this->dataPtr->AddSsao(cam);
  }

  this->dataPtr->gBufSchemeHandler = new SsaoGBufferSchemeHandler();
  Ogre::MaterialManager::getSingleton().addListener(
      this->dataPtr->gBufSchemeHandler, "GBuffer");
}

/////////////////////////////////////////////////
void AmbientOcclusionVisualPluginPrivate::AddSsao(rendering::CameraPtr _cam)
{
  Ogre::Viewport *ogreViewport = _cam->OgreCamera()->getViewport();
  if (!ogreViewport)
  {
    gzerr << "Viewport is null. Ambient Occlusion will not be enabled"
          << std::endl;
    return;
  }

  // GBuffer
  Ogre::CompositorInstance *gBufCompInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(
      ogreViewport, "SSAO/GBuffer");
  if (gBufCompInstance)
  {
    gBufCompInstance->setEnabled(true);
    // the GBuffer compositor should have 2 passes,
    // see gazebo/media/materials/scripts/CreaseShading.compositor
    if (gBufCompInstance->getTechnique()->getNumTargetPasses() > 1)
    {
      // set a visibility mask so that the ssao effect does not apply to
      // gui visuals
      gBufCompInstance->getTechnique()->getTargetPass(1)->setVisibilityMask(
          GZ_VISIBILITY_ALL & ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
    }
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
