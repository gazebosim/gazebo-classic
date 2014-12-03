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
#ifndef _NULLSCHEMEHANDLER_HH_
#define _NULLSCHEMEHANDLER_HH_

#include <OgreMaterialManager.h>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Class for skipping materials which do not have the scheme defined
    class NullSchemeHandler :
      public Ogre::MaterialManager::Listener
    {
      /** @copydoc MaterialManager::Listener::handleSchemeNotFound */
      public: virtual Ogre::Technique *handleSchemeNotFound(
                  uint16_t /*_schemeIndex*/,
                  const Ogre::String &_schemeName,
                  Ogre::Material *_originalMaterial,
                  uint16_t /*_lodIndex*/,
                  const Ogre::Renderable * /*_rend*/)
        {
          // Creating a technique so the handler only gets called
          // once per material
          Ogre::Technique *emptyTech = _originalMaterial->createTechnique();
          emptyTech->removeAllPasses();
          emptyTech->setSchemeName(_schemeName);
          return emptyTech;
        }
    };
  }
}

#endif
