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
#ifndef _DEFERRED_LIGHT_CP_HH_
#define _DEFERRED_LIGHT_CP_HH_

#include <OgreCompositorInstance.h>
#include <OgreCustomCompositionPass.h>

#include <map>

#include "gazebo/rendering/deferred_shading/DeferredLight.hh"
#include "gazebo/rendering/deferred_shading/MaterialGenerator.hh"
#include "gazebo/rendering/deferred_shading/AmbientLight.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief The render operation that will be called each frame in the custom
    /// composition pass This is the class that will send the actual render
    /// calls of the spheres (point lights), cones (spotlights) and quads
    /// (directional lights) after the GBuffer has been constructed
    class DeferredLightRenderOperation
      : public Ogre::CompositorInstance::RenderSystemOperation
    {
      public: DeferredLightRenderOperation(Ogre::CompositorInstance *_instance,
                                           const Ogre::CompositionPass *_pass);

      /// @copydoc CompositorInstance::RenderSystemOperation::execute
      public: virtual void execute(Ogre::SceneManager *_sm,
                                   Ogre::RenderSystem *_rs);

      public: virtual ~DeferredLightRenderOperation();

      /// \brief Create a new deferred light
      private: DeferredLight *CreateDeferredLight(Ogre::Light *_light);

      /// The texture names of the GBuffer components
      private: Ogre::String texName0;
      private: Ogre::String texName1;

      /// The material generator for the light geometry
      private: MaterialGenerator *lightMaterialGenerator;

      /// The map of deferred light geometries already constructed
      private: typedef std::map<Ogre::Light*, DeferredLight*> LightsMap;
      private: LightsMap lights;

      /// The ambient light used to render the scene
      private: AmbientLight *ambientLight;

      /// The viewport that we are rendering to
      private: Ogre::Viewport* viewport;
    };

    /// \brief The custom composition pass that is used for rendering the light
    /// geometry. This class needs to be registered with the CompositorManager
    class DeferredLightCompositionPass : public Ogre::CustomCompositionPass
    {
      /// @copydoc CustomCompositionPass::createOperation
      public: virtual Ogre::CompositorInstance::RenderSystemOperation
              *createOperation(Ogre::CompositorInstance *_instance,
                              const Ogre::CompositionPass *_pass)
        {
          return OGRE_NEW DeferredLightRenderOperation(_instance, _pass);
        }

      protected: virtual ~DeferredLightCompositionPass() {}
    };
  }
}
#endif
