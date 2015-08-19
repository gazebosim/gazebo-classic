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

#include <string>

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
      /// \enum Modes
      /// \brief Visual is in one of these modes at each time.
      public: enum Mode
      {
        /// \brief None
        NONE = 0,
        /// \brief Force
        FORCE = 1,
        /// \brief Torque
        TORQUE = 2
      };

      /// \brief Constructor
      /// \param[in] _name Name of the visual
      /// \param[in] _parentVis Pointer to the parent visual
      public: ApplyWrenchVisual(const std::string &_name, VisualPtr _parentVis);

      /// \brief Destructor
      public: virtual ~ApplyWrenchVisual();

      // Documentation inherited.
      public: void Load();

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief Set the CoM vector and update the position of the torque
      /// visual.
      /// \param[in] _comVector New vector.
      public: void SetCoM(const math::Vector3 &_comVector);

      /// \brief Set the force position vector and update the position of the
      /// force visual.
      /// \param[in] _forcePosVector New vector.
      public: void SetForcePos(const math::Vector3 &_forcePosVector);

      /// \brief Update force vector, force text and mode.
      /// \param[in] _forceVector New vector.
      /// \param[in] _rotatedByMouse Whether the rotation comes from the mouse
      /// or not.
      public: void SetForce(const math::Vector3 &_forceVector,
          const bool _rotatedByMouse);

      /// \brief Update torque vector, torque text and mode.
      /// \param[in] _torqueVector New vector.
      /// \param[in] _rotatedByMouse Whether the rotation comes from the mouse
      /// or not.
      public: void SetTorque(const math::Vector3 &_torqueVector,
          const bool _rotatedByMouse);

      /// \brief Update the force visual according to the force and force
      /// position vectors.
      public: void UpdateForceVisual();

      /// \brief Update the torque visual according to the torque and CoM
      /// vectors.
      public: void UpdateTorqueVisual();

      /// \brief Resize all children according to target link's size.
      public: void Resize();

      /// \brief Returns the force visual.
      /// \return Pointer to force visual.
      public: rendering::VisualPtr GetForceVisual() const;

      /// \brief Returns the torque visual.
      /// \return Pointer to torque visual.
      public: rendering::VisualPtr GetTorqueVisual() const;

      /// \brief Returns the rotation tool.
      /// \return Pointer to rotation tool.
      public: rendering::SelectionObjPtr GetRotTool() const;

      /// \brief Set the mode to force, torque or none, update colors
      /// and visibility accordingly.
      /// \param[in] _mode New mode.
      public: void SetMode(Mode _mode);

      /// \brief Get the rotation to point the positive Z axis to the
      /// given direction.
      /// \param[in] _dir Direction vector.
      /// \return Resulting quaternion
      private: math::Quaternion GetQuaternionFromVector(
          const math::Vector3 &_dir);
    };
    /// \}
  }
}
#endif
