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

#ifndef _JOINTVISUAL_HH_
#define _JOINTVISUAL_HH_

#include <string>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class JointVisual JointVisual.hh rendering/rendering.hh
    /// \brief Visualization for joints
    class GAZEBO_VISIBLE JointVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the visual
      /// \param[in] _vis Pointer to the parent visual
      public: JointVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor
      public: virtual ~JointVisual();

      /// \brief Load the joint visual based on a message
      /// \param[in] _msg Joint message
      public: void Load(ConstJointPtr &_msg);
      using Visual::Load;

      /// \internal
      /// \brief Load the joint visual based on a message and an offset pose
      /// This is currently used internally for creating a second visual for
      /// joint types that have more than 1 axis.
      /// \param[in] _msg Joint message
      /// \param[in] _pose Pose of the joint visual in world coordinates.
      public: void Load(ConstJointPtr &_msg, const math::Pose &_worldPose);

      /// \brief Create an axis and attach it to the joint visual.
      /// \param[in] _axis Axis vector
      /// \param[in] _useParentFrame True to use parent frame instead of the
      /// joint frame.
      /// \param[in] _type Type of axis.
      public: void CreateAxis(const math::Vector3 &_axis, bool _useParentFrame,
          msgs::Joint::Type _type);

      // Documentation Inherited.
      public: void SetVisible(bool _visible, bool _cascade = true);
    };
    /// \}
  }
}
#endif
