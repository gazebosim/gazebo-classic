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

#ifndef _JOINTVISUAL_PRIVATE_HH_
#define _JOINTVISUAL_PRIVATE_HH_

#include <string>
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Joint Visual class.
    class JointVisualPrivate : public VisualPrivate
    {
      /// \brief The joint's XYZ frame visual.
      public: AxisVisualPtr axisVisual;

      /// \brief The visual representing the one joint axis. There can be only
      /// one axis visual per joint visual, so joints with two axes have a 2nd
      /// JointVisual with its own arrowVisual.
      public: ArrowVisualPtr arrowVisual;

      /// \brief Second joint visual for hinge2 and universal joints. It is a
      /// simplified visual without an XYZ frame.
      public: JointVisualPtr parentAxisVis;

      /// \brief Scale based on the size of the joint's child link.
      public: math::Vector3 scaleToLink;
    };
  }
}
#endif
