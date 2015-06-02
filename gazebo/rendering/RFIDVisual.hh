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

#ifndef _RFIDVISUAL_HH_
#define _RFIDVISUAL_HH_

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

    /// \class RFIDVisual RFIDVisual.hh rendering/rendering.hh
    /// \brief Visualization for RFID sensor.
    class GZ_RENDERING_VISIBLE RFIDVisual : public Visual
    {
      /// \brief Constructor.
      /// \param[in] _name Name of the Visual.
      /// \param[in] _vis Parent Visual.
      /// \param[in] _topicName Name of the topic which publishes RFID data.
      public: RFIDVisual(const std::string &_name, VisualPtr _vis,
                         const std::string &_topicName);

      /// \brief Destructor
      public: virtual ~RFIDVisual();

      /// \brief Callback for the RFID sensor data.
      /// \param[in] _msg Message containing RFID data
      private: void OnScan(ConstPosePtr &_msg);
    };
    /// \}
  }
}
#endif
