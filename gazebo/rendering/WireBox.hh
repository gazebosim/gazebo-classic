/*
 * Copyright 2011 Nate Koenig
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
#ifndef _WIREBOUNDINGBOX_HH_
#define _WIREBOUNDINGBOX_HH_

#include <string>
#include "gazebo/math/Box.hh"
#include "gazebo/rendering/DynamicLines.hh"

namespace gazebo
{
  namespace rendering
  {
    class WireBoundingBox
    {
      /// \brief Constructor
      public: explicit WireBoundingBox(const math::Box &_box);

      /// \brief Destructor
      public: ~WireBoundingBox();

      /// \brief Builds the wireframe line list.
      /// \param[in] _box Bounding box to build a wireframe from.
      public: void Init(const math::Box &_box);

      private: DynamicLines *lines;
    };
  }
}
#endif
