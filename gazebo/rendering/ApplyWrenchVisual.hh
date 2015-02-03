/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _APPLYWRENCHVISUAL_HH_
#define _APPLYWRENCHVISUAL_HH_

//#include <string>

//#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/Visual.hh"
//#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class ApplyWrenchVisual ApplyWrenchVisual.hh rendering/rendering.hh
    /// \brief Visualization for the apply wrench GUI
    class GAZEBO_VISIBLE ApplyWrenchVisual : public Visual
    {
      /// \enum WrenchModes
      /// \brief Identifies if either in force mode or torque mode.
      public: enum WrenchModes {
                  /// \brief Force mode
                  FORCE,
                  /// \brief Torque mode
                  TORQUE
                };

      /// \brief Constructor
      /// \param[in] _name Name of the visual
      /// \param[in] _parentVis Pointer to the parent visual
      public: ApplyWrenchVisual(const std::string &_name, VisualPtr _parentVis);

      /// \brief Destructor
      public: virtual ~ApplyWrenchVisual();

      /// \brief TODO
      public: void Load();

      /// \brief TODO
      public: void SetMode(WrenchModes _mode);

      /// \brief TODO
      public: void UpdateForce(math::Vector3 _forceVector);

      /// \brief TODO
      public: void UpdateTorque(math::Vector3 _torqueVector);

      /// \brief TODO
      public: ApplyWrenchVisual::WrenchModes wrenchMode;
    };
    /// \}
  }
}
#endif
