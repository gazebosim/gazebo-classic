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
#ifndef _MERGE_CP_HH_
#define _MERGE_CP_HH_

#include <OgreCompositorInstance.h>
#include <OgreCustomCompositionPass.h>
#include "gazebo/util/system.hh"


namespace gazebo
{
  namespace rendering
  {
    /// The render operation that will be called each frame in the custom
    // composition pass. This is the class that will send the actual render
    // calls of the spheres (point lights), cones (spotlights) and quads
    // (directional lights) after the GBuffer has been constructed
    class MergeRenderOperation :
      public Ogre::CompositorInstance::RenderSystemOperation
    {
      public: MergeRenderOperation(Ogre::CompositorInstance *_instance,
                                   const Ogre::CompositionPass *_pass);

      /// @copydoc CompositorInstance::RenderSystemOperation::execute
      public: virtual void execute(Ogre::SceneManager *_sm,
                                   Ogre::RenderSystem *_rs);

      public: virtual ~MergeRenderOperation();
    };

    /// The custom composition pass that is used for rendering the light
    /// geometry. This class needs to be registered with the CompositorManager
    class MergeCompositionPass :
      public Ogre::CustomCompositionPass
    {
      /// @copydoc CustomCompositionPass::createOperation
      public: virtual Ogre::CompositorInstance::RenderSystemOperation *
              CreateOperation(Ogre::CompositorInstance *_instance,
                              const Ogre::CompositionPass *_pass)
      {
        return OGRE_NEW MergeRenderOperation(instance, pass);
      }

      protected: virtual ~MergeCompositionPass() {}
    };
  }
}
#endif
