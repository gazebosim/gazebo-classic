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

#ifndef _COMVISUAL_HH_
#define _COMVISUAL_HH_

#include <string>

#include "gazebo/math/Pose.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class COMVisual COMVisual.hh rendering/rendering.hh
    /// \brief Basic Center of Mass visualization
    class GZ_RENDERING_VISIBLE COMVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the Visual
      /// \param[in] _vis Parent Visual
      public: COMVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Load the Visual from an SDF pointer
      /// \param[in] _elem SDF Element pointer
      public: virtual void Load(sdf::ElementPtr _elem);
      using Visual::Load;

      /// \brief Load from a message
      /// \param[in] _msg Pointer to the message
      public: virtual void Load(ConstLinkPtr &_msg);

      /// \brief Get inertia pose.
      /// \return Inertia pose in link frame.
      public: math::Pose GetInertiaPose() const;

      /// \brief Load using previously set member variables.
      private: void Load();
    };
    /// \}
  }
}
#endif
