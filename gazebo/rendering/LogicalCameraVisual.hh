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
#ifndef _GAZEBO_LOGICAL_CAMERAVISUAL_HH_
#define _GAZEBO_LOGICAL_CAMERAVISUAL_HH_

#include <string>

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \brief Logical camera visualization
    ///
    /// This class is used to visualize a logical camera generated from
    /// a LogicalCameraSensor. The sensor's frustum is drawn in the 3D
    /// environment.
    class GZ_RENDERING_VISIBLE LogicalCameraVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the Visual
      /// \param[in] _vis Pointer to the parent Visual
      public: LogicalCameraVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor
      public: virtual ~LogicalCameraVisual();

      /// \brief Load the Visual
      /// \param[in] _msg Message describing the camera sensor.
      public: void Load(const msgs::LogicalCameraSensor &_msg);
      using Visual::Load;

      // Documentation inherited
      protected: virtual void Fini();

      /// \brief Update the visual
      private: void Update();
    };
    /// \}
  }
}
#endif
