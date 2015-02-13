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

#ifndef _GAZEBO_APPLYWRENCHVISUAL_HH_
#define _GAZEBO_APPLYWRENCHVISUAL_HH_

#include "gazebo/rendering/Visual.hh"

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
      /// \brief Constructor
      /// \param[in] _name Name of the visual
      /// \param[in] _parentVis Pointer to the parent visual
      public: ApplyWrenchVisual(const std::string &_name, VisualPtr _parentVis);

      /// \brief Destructor
      public: virtual ~ApplyWrenchVisual();

      /// \brief TODO
      public: void Load();

      /// \brief TODO
      public: rendering::VisualPtr GetForceVisual() const;

      /// \brief TODO
      public: rendering::VisualPtr GetTorqueVisual() const;

      /// \brief TODO
      public: rendering::SelectionObjPtr GetRotTool() const;

      /// \brief TODO
      public: math::Quaternion GetQuaternionFromVector(math::Vector3 _vec);

      /// \brief TODO
      public: void SetWrenchMode(std::string _mode);

      /// \brief TODO
      public: void SetCoM(math::Vector3 _comVector);

      /// \brief TODO
      public: void SetForcePos(math::Vector3 _forcePosVector);

      /// \brief TODO
      public: void SetForce(math::Vector3 _forceVector, bool _rotatedByMouse);

      /// \brief TODO
      public: void SetTorque(math::Vector3 _torqueVector, bool _rotatedByMouse);

      /// \brief TODO
      public: void SetForceVisual();

      /// \brief TODO
      public: void SetTorqueVisual();

      // Documentation Inherited.
      public: void SetVisible(bool _visible, bool _cascade = false);
    };
    /// \}
  }
}
#endif
