/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_LINK_FRAME_VISUAL_HH_
#define _GAZEBO_LINK_FRAME_VISUAL_HH_

#include <string>

#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/AxisVisual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class LinkFrameVisual LinkFrameVisual.hh rendering/rendering.hh
    /// \brief Visualization for link frames.
    class GZ_RENDERING_VISIBLE LinkFrameVisual : public AxisVisual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the LinkFrameVisual
      /// \param[in] _parent Parent visual
      public: LinkFrameVisual(const std::string &_name, VisualPtr _parent);

      /// \brief Destructor
      public: virtual ~LinkFrameVisual() = default;

      // Documentation inherited
      public: virtual void Load();

      // Documentation inherited
      public: virtual void SetHighlighted(bool _highlighted);

      // Documentation inherited
      public: bool GetHighlighted();
    };
    /// \}
  }
}
#endif
