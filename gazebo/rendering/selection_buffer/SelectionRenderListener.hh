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
#ifndef _SELECTIONRENDERLISTENER_HH_
#define _SELECTIONRENDERLISTENER_HH_

#include "gazebo/rendering/ogre_gazebo.h"

namespace gazebo
{
  namespace rendering
  {
    class MaterialSwitcher;
    // We need this attached to the depth target, otherwise we get problems with
    // the compositor MaterialManager.Listener should NOT be running all the
    // time - rather only when we're specifically rendering the target that
    // needs it
    class SelectionRenderListener : public Ogre::RenderTargetListener
    {
      /// \brief Constructor
      public: SelectionRenderListener(MaterialSwitcher *_switcher);

      /// \brief Destructor
      public: ~SelectionRenderListener();

      public: virtual void preRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &_evt);

      public: virtual void postRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &_evt);

      private: MaterialSwitcher *materialListener;
    };
  }
}
#endif
