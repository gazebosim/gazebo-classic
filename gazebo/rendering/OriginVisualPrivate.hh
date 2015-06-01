/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_ORIGIN_VISUAL_PRIVATE_HH_
#define _GAZEBO_ORIGIN_VISUAL_PRIVATE_HH_

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
    class DynamicLines;

    /// \brief Private data for the Origin Visual class
    class OriginVisualPrivate : public VisualPrivate
    {
      /// \brief Length of lines
      public: double length;

      /// \brief Line on the X axis.
      public: DynamicLines *xLine;

      /// \brief Line on the Y axis.
      public: DynamicLines *yLine;

      /// \brief Line on the Z axis.
      public: DynamicLines *zLine;
    };
  }
}
#endif
