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
/* Desc: Center of Mass Visualization Class
 * Author: Nate Koenig
 */

#ifndef COMVISUAL_HH
#define COMVISUAL_HH

#include <string>

#include "rendering/Visual.hh"
#include "msgs/msgs.h"

namespace ogre
{
  class SceneNode;
}

namespace gazebo
{
  namespace rendering
  {
    class DynamicLines;
    class COMVisual : public Visual
    {
      public: COMVisual(const std::string &_name, VisualPtr _vis);
      public: virtual ~COMVisual();

      public: virtual void Load(ConstLinkPtr &_msg);
      private: DynamicLines *crossLines;
      private: Ogre::SceneNode *boxNode;
    };
  }
}
#endif
