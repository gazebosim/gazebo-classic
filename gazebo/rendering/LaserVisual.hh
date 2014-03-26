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

#ifndef _LASERVISUAL_HH_
#define _LASERVISUAL_HH_

#include <string>

#include "gazebo/common/Color.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class LaserVisual LaserVisual.hh rendering/rendering.hh
    /// \brief Visualization for laser data.
    class GAZEBO_VISIBLE LaserVisual : public Visual
    {
      /// \brief Constructor.
      /// \param[in] _name Name of the visual.
      /// \param[in] _vis Pointer to the parent Visual.
      /// \param[in] _topicName Name of the topic that has laser data.
      public: LaserVisual(const std::string &_name, VisualPtr _vis,
                          const std::string &_topicName);

      /// \brief Destructor.
      public: virtual ~LaserVisual();

      /// Documentation inherited from parent.
      public: virtual void SetEmissive(const common::Color &_color);

      /// \brief Callback when laser data is received.
      private: void OnScan(ConstLaserScanStampedPtr &_msg);

      /// \brief Update the Visual
      private: void Update();
    };
    /// \}
  }
}
#endif
