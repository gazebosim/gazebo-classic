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

#ifndef _TRANSMITTERVISUAL_HH_
#define _TRANSMITTERVISUAL_HH_

#include <string>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/Visual.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class TransmitterVisual TransmitterVisual.hh rendering/rendering.hh
    /// \brief Visualization for the wireless propagation data.
    class GZ_RENDERING_VISIBLE TransmitterVisual : public Visual
    {
      /// \brief Constructor.
      /// \param[in] _name Name of the visual.
      /// \param[in] _vis Pointer to the parent Visual.
      /// \param[in] _topicName Name of the topic that has laser data.
      public: TransmitterVisual(const std::string &_name, VisualPtr _vis,
                                const std::string &_topicName);

      /// \brief Destructor.
      public: virtual ~TransmitterVisual();

      /// Documentation inherited from parent.
      public: virtual void Load();

      /// \brief Function that runs on the OGRE thread to refresh the UI.
      public: virtual void Update();

      /// \brief Callback when a new propagation grid is received
      /// \brief[in] _msg New transmitter propagation grid received
      private: void OnNewPropagationGrid(ConstPropagationGridPtr &_msg);
    };
    /// \}
  }
}
#endif
