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
/* Desc: Joint Visualization Class
 * Author: Nate Koenig
 */

#ifndef _JOINTVISUAL_HH_
#define _JOINTVISUAL_HH_

#include <string>
#include "gazebo/rendering/Visual.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class JointVisual JointVisual.hh rendering/rendering.hh
    /// \brief Visualization for joints
    class JointVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the visual
      /// \param[in] _vis Pointer to the parent visual
      public: JointVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor
      public: virtual ~JointVisual();

      /// \brief Load the visual based on a message
      /// \param[in] _msg Joint message
      public: void Load(ConstJointPtr &_msg);

      /// \brief The visual used to draw the joint.
      private: AxisVisualPtr axisVisual;
    };
    /// \}
  }
}
#endif
