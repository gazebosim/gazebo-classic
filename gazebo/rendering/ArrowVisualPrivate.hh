/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _ARROWVISUAL_PRIVATE_HH_
#define _ARROWVISUAL_PRIVATE_HH_

#include <string>
#include "gazebo/rendering/VisualPrivate.hh"

namespace ogre
{
  class SceneNode;
}

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Arrow Visual class
    class ArrowVisualPrivate : public VisualPrivate
    {
      /// \brief Head scene node.
      public: Ogre::SceneNode *headNode;

      /// \brief Shaft scene node.
      public: Ogre::SceneNode *shaftNode;

      /// \brief Rotation scene node.
      public: Ogre::SceneNode *rotationNode;

      /// \brief Head node visible flag.
      public: bool headNodeVisible;

      /// \brief Shaft node visible flag.
      public: bool shaftNodeVisible;

      /// \brief Rotation node visible flag.
      public: bool rotationNodeVisible;
    };
  }
}
#endif
