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
/* Desc: Center of Mass Visualization Class
 * Author: Nate Koenig
 */

#ifndef _COMVISUAL_HH_
#define _COMVISUAL_HH_

#include <string>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/msgs/msgs.hh"

namespace ogre
{
  class SceneNode;
}

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    class DynamicLines;

    /// \class COMVisual COMVisual.hh rendering/rendering.hh
    /// \brief Basic Center of Mass visualization
    class COMVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the Visual
      /// \param[in] _vis Parent Visual
      public: COMVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor
      public: virtual ~COMVisual();

      /// \brief Load the Visual from an SDF pointer
      /// \param[in] _elem SDF Element pointer
      public: virtual void Load(sdf::ElementPtr _elem);

      /// \brief Load from a message
      /// \param[in] _msg Pointer to the message
      public: virtual void Load(ConstLinkPtr &_msg);

      /// \brief Load based on a math::Pose
      /// \param[in] _pose Pose of the COM visual
      /// \param[in] _scale Scale factor for the COM visual.
      private: void Load(const math::Pose &_pose,
               const math::Vector3 &_scale = math::Vector3(0.02, 0.02, 0.02));

      /// \brief Lines that make the cross marking the center of mass
      private: DynamicLines *crossLines;

      /// \brief Box that make the cross marking the center of mass
      private: Ogre::SceneNode *boxNode;
    };
    /// \}
  }
}
#endif
