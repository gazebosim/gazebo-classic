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

#ifndef _APPLYWRENCHVISUAL_PRIVATE_HH_
#define _APPLYWRENCHVISUAL_PRIVATE_HH_

//#include <string>
//#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Apply Wrench Visual class.
    class ApplyWrenchVisualPrivate : public VisualPrivate
    {
      /// \brief TODO
      public: VisualPtr comVisual;

      /// TODO
      public: rendering::ArrowVisualPtr forceVisual;

      /// TODO
      public: rendering::SelectionObjPtr rotTool;

      /// \brief TODO
      public: VisualPtr torqueVisual;

      /// TODO
      public: math::Vector3 comVector;

      /// TODO
      public: math::Vector3 forcePosVector;

      /// TODO
      public: math::Vector3 forceVector;

      /// TODO
      public: math::Vector3 torqueVector;

      /// TODO
      public: std::string mode;

      /// TODO
      public: bool rotatedByMouse;
    };
  }
}
#endif
