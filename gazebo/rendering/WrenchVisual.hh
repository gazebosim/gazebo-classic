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

#ifndef _WRENCHVISUAL_HH_
#define _WRENCHVISUAL_HH_

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

    /// \class WrenchVisual WrenchVisual.hh rendering/rendering.hh
    /// \brief Visualization for sonar data.
    class GAZEBO_VISIBLE WrenchVisual : public Visual
    {
      /// \brief Constructor.
      /// \param[in] _name Name of the visual.
      /// \param[in] _vis Pointer to the parent Visual.
      /// \param[in] _topicName Name of the topic that has sonar data.
      public: WrenchVisual(const std::string &_name, VisualPtr _vis,
                          const std::string &_topicName);

      /// \brief Destructor.
      public: virtual ~WrenchVisual();

      /// \brief Load the visual based on a message
      /// \param[in] _msg Joint message
      public: void Load(ConstJointPtr &_msg);
      using Visual::Load;

      /// \brief Set to true to enable wrench visualization.
      /// \param[in] _enabled True to show wrenches, false to hide.
      public: void SetEnabled(bool _enabled);

      /// \brief Callback when sonar data is received.
      private: void OnMsg(ConstWrenchStampedPtr &_msg);

      /// \brief Update the wrench visual.
      private: void Update();
    };
    /// \}
  }
}
#endif
