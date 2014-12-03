/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _DEFERRED_LIGHT_CP_HH_
#define _DEFERRED_LIGHT_CP_HH_

#include <OgreCompositorInstance.h>
#include <OgreCustomCompositionPass.h>

#include <map>
#include <vector>

#include "gazebo/rendering/deferred_shading/LightMaterialGenerator.hh"
#include "gazebo/rendering/deferred_shading/TechniqueDefinitions.hh"
#include "gazebo/rendering/deferred_shading/DeferredLight.hh"
#include "gazebo/rendering/deferred_shading/MaterialGenerator.hh"
#include "gazebo/rendering/deferred_shading/AmbientLight.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief The render operation that will be called each frame in the custom
    /// composition pass This is the class that will send the actual render
    /// calls of the spheres (point lights), cones (spotlights) and quads
    /// (directional lights) after the GBuffer has been constructed
    template<typename techniquePolicy>
    class DeferredLightRenderOperation
      : public Ogre::CompositorInstance::RenderSystemOperation,
        public techniquePolicy
    {
      public: DeferredLightRenderOperation(Ogre::CompositorInstance *_instance,
                                           const Ogre::CompositionPass *_pass)
      {
        this->viewport = _instance->getChain()->getViewport();

        /// Get the names of the GBuffer textures
        for (int i = 0; i < this->GetGBufferSize(); ++i)
        {
          const Ogre::CompositionPass::InputTex &input = _pass->getInput(i);
          this->inputTexNames.push_back(
              _instance->getTextureInstanceName(input.name, input.mrtIndex));
        }

        // Create lights material generator
        this->lightMaterialGenerator =
          new LightMaterialGenerator<techniquePolicy>();

        // Create the ambient light
        this->ambientLight = new AmbientLight<techniquePolicy>();
        const Ogre::MaterialPtr &mat = this->ambientLight->getMaterial();
        mat->load();
        this->instanceManager = NULL;
        this->rsmActive = false;
      }

      /// @copydoc CompositorInstance::RenderSystemOperation::execute
      public: virtual void execute(Ogre::SceneManager *_sm,
                                   Ogre::RenderSystem * /*_rs*/)
      {
        if (!this->instanceManager)
        {
          this->instanceManager = _sm->getInstanceManager("VPL_InstanceMgr");
        }

        Ogre::Camera *cam = this->viewport->getCamera();
        Ogre::Technique *tech;

        // Update the ambient light based on the camera
        this->ambientLight->UpdateFromCamera(cam);

        tech = this->ambientLight->getMaterial()->getBestTechnique();
        InjectTechnique(_sm, tech, this->ambientLight, 0);

        // TODO: Improve this.
        bool findVisible = _sm->getFindVisibleObjects();
        _sm->setFindVisibleObjects(true);

        const Ogre::LightList &lightList = _sm->_getLightsAffectingFrustum();
        for (Ogre::LightList::const_iterator it = lightList.begin();
             it != lightList.end(); ++it)
        {
          Ogre::Light *light = *it;
          Ogre::LightList ll;
          ll.push_back(light);

          LightsMap::iterator dLightIt = this->lights.find(light);
          DeferredLight* dLight = 0;
          if (dLightIt == this->lights.end())
          {
            dLight = this->CreateDeferredLight(light);
            if (dLight->getCastShadows())
            {
              dLight->SetVPLCount(400, _sm, this->instanceManager);
            }
          }
          else
          {
            dLight = dLightIt->second;
            dLight->UpdateFromParent();
          }

          dLight->UpdateFromCamera(cam);
          tech = dLight->getMaterial()->getBestTechnique();

          // Update shadow texture
          if (dLight->getCastShadows())
          {
            Ogre::SceneManager::RenderContext *context = _sm->_pauseRendering();

            _sm->prepareShadowTextures(cam, this->viewport, &ll);
            _sm->_resumeRendering(context);

            const Ogre::TexturePtr &shadowTex = _sm->getShadowTexture(0);
            dLight->UpdateRSM(shadowTex);

            Ogre::ShadowCameraSetupPtr cameraSetup =
              dLight->GetParentLight()->getCustomShadowCameraSetup();
            if (cameraSetup.isNull())
            {
              cameraSetup = _sm->getShadowCameraSetup();
            }

            /*Ogre::Camera shadowCam("temp_shadow_cam", _sm);
            shadowCam.setAspectRatio(1.0);

            cameraSetup->getShadowCamera(_sm, cam, this->viewport,
                dLight->GetParentLight(), &shadowCam, 0);
            Ogre::Matrix4 proj = shadowCam.getProjectionMatrix();
            Ogre::Matrix4 view = shadowCam.getViewMatrix();
            proj = proj*view;
            Ogre::Matrix4 invProj = proj.inverse();
            dLight->UpdateShadowInvProj(invProj);
            */

            if (this->rsmActive)
            {
              dLight->RenderVPLs(_sm, this->instanceManager);
            }

            Ogre::Pass *pass = tech->getPass(0);
            Ogre::TextureUnitState *tus =
              pass->getTextureUnitState("ShadowMap");

            if (!tus)
              gzthrow("Invalid texture unit state");

             if (tus->_getTexturePtr() != shadowTex)
              tus->_setTexturePtr(shadowTex);
          }
          else
            printf("NO SHADOWS\n");

          InjectTechnique(_sm, tech, dLight, &ll);
        }

        // restore previous settings
        _sm->setFindVisibleObjects(findVisible);
      }

      public: virtual ~DeferredLightRenderOperation()
      {
        for (LightsMap::iterator it = this->lights.begin();
             it != this->lights.end(); ++it)
        {
          delete it->second;
        }
        this->lights.clear();

        delete this->ambientLight;
        delete this->lightMaterialGenerator;
      }

      /// \brief Create a new deferred light
      private: DeferredLight *CreateDeferredLight(Ogre::Light *_light)
      {
        DeferredLight *rv = new DeferredLight(this->lightMaterialGenerator,
            _light, Ogre::MaterialManager::getSingleton().getByName(
              this->GetMaterialPrefix()+"/VPL"));
        this->lights[_light] = rv;
        return rv;
      }

      private: void InjectTechnique(Ogre::SceneManager *_sm,
                   Ogre::Technique *_tech, Ogre::Renderable *_rend,
                   const Ogre::LightList *_lightList)
      {
        for (uint16_t i = 0; i < _tech->getNumPasses(); ++i)
        {
          Ogre::Pass *pass = _tech->getPass(i);

          if (_lightList != 0)
            _sm->_injectRenderWithPass(pass, _rend, false, false, _lightList);
          else
            _sm->_injectRenderWithPass(pass, _rend, false);
        }
      }

      /// The texture names of the GBuffer components
      private: std::vector<Ogre::String> inputTexNames;

      /// The material generator for the light geometry
      private: MaterialGenerator *lightMaterialGenerator;

      /// The map of deferred light geometries already constructed
      private: typedef std::map<Ogre::Light*, DeferredLight*> LightsMap;
      private: LightsMap lights;

      /// The ambient light used to render the scene
      private: AmbientLight<techniquePolicy> *ambientLight;

      /// The viewport that we are rendering to
      private: Ogre::Viewport* viewport;

      private: Ogre::InstanceManager *instanceManager;
      private: bool rsmActive;
    };

    /// \brief The custom composition pass that is used for rendering the light
    /// geometry. This class needs to be registered with the CompositorManager
    template<typename techniquePolicy>
    class DeferredLightCompositionPass
      : public Ogre::CustomCompositionPass, public techniquePolicy
    {
      /// @copydoc CustomCompositionPass::createOperation
      public: virtual Ogre::CompositorInstance::RenderSystemOperation
              *createOperation(Ogre::CompositorInstance *_instance,
                              const Ogre::CompositionPass *_pass)
        {
          return OGRE_NEW DeferredLightRenderOperation<techniquePolicy>(
              _instance, _pass);
        }

      protected: virtual ~DeferredLightCompositionPass() {}
    };
  }
}
#endif
