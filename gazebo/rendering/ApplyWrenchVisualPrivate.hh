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

#ifndef _GAZEBO_APPLYWRENCHVISUAL_PRIVATE_HH_
#define _GAZEBO_APPLYWRENCHVISUAL_PRIVATE_HH_

#include <string>
#include <mutex>

#include "gazebo/rendering/VisualPrivate.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Apply Wrench Visual class.
    class ApplyWrenchVisualPrivate : public VisualPrivate
    {
      /// \brief Material for the current mode.
      public: std::string selectedMaterial;

      /// \brief Material for the unselected mode.
      public: std::string unselectedMaterial;

      /// \brief Arrow representing force.
      public: VisualPtr forceVisual;

      /// \brief Tube and line representing torque.
      public: VisualPtr torqueVisual;

      /// \brief Line connecting the torque visual to the CoM.
      public: rendering::DynamicLines *torqueLine;

      /// \brief CoM position in link coordinates.
      public: ignition::math::Vector3d comVector;

      /// \brief Force application point in link coordinates.
      public: ignition::math::Vector3d forcePosVector;

      /// \brief Force vector expressed in the link frame.
      public: ignition::math::Vector3d forceVector;

      /// \brief Torque vector expressed in the link frame.
      public: ignition::math::Vector3d torqueVector;

      /// \brief Mutex to protect variables
      public: std::mutex mutex;

      /// \brief Rotation tool composed of two circles.
      public: rendering::SelectionObjPtr rotTool;

      /// \brief If true, the rotation tool was rotated by the mouse and
      /// shouldn't be oriented again according to the vector.
      public: bool rotatedByMouse;

      /// \brief Text displaying the force magnitude.
      public: MovableText forceText;

      /// \brief Text displaying the torque magnitude.
      public: MovableText torqueText;
    };
  }
}
#endif
