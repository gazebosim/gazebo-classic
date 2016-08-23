/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _LISTENER_FACTORY_LOGIC_HH_
#define _LISTENER_FACTORY_LOGIC_HH_

#include <OgreCompositorInstance.h>
#include <OgreCompositorLogic.h>
#include <map>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// The simple types of compositor logics will all do the same thing -
    /// Attach a listener to the created compositor
    class GZ_RENDERING_VISIBLE ListenerFactoryLogic :
      public Ogre::CompositorLogic
    {
      //// @copydoc CompositorLogic::compositorInstanceCreated
      public: virtual void compositorInstanceCreated(
                  Ogre::CompositorInstance *_newInstance)
      {
        Ogre::CompositorInstance::Listener *listener =
          createListener(_newInstance);

        _newInstance->addListener(listener);
        this->listeners[_newInstance] = listener;
      }

      /// @copydoc CompositorLogic::compositorInstanceDestroyed
      public: virtual void compositorInstanceDestroyed(
                  Ogre::CompositorInstance *_destroyedInstance)
      {
        delete this->listeners[_destroyedInstance];
        this->listeners.erase(_destroyedInstance);
      }

      // This is the method that implementers will need to create
      protected: virtual Ogre::CompositorInstance::Listener *createListener(
                     Ogre::CompositorInstance *instance) = 0;

      private: typedef std::map<Ogre::CompositorInstance*,
               Ogre::CompositorInstance::Listener*> ListenerMap;

      ListenerMap listeners;
    };
  }
}
#endif
