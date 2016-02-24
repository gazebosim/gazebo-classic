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

#ifndef _GAZEBO_GUI_BUILDING_MODEL_MANIP_PRIVATE_HH_
#define _GAZEBO_GUI_BUILDING_MODEL_MANIP_PRIVATE_HH_

#include <string>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Color.hh"
#include "gazebo/common/CommonTypes.hh"

#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare pointers.
    class BuildingMaker;

    /// \internal
    /// \brief Private data for the BuildingModelManip class
    class BuildingModelManipPrivate
    {
      /// \brief Name of the manip.
      public: std::string name;

      /// \brief A pointer to the visual managed by the manip.
      public: rendering::VisualPtr visual;

      /// \brief Size of the manipular.
      public: ignition::math::Vector3d size;

      /// \brief Pose of the manip.
      public: ignition::math::Pose3d pose;

      /// \brief Maker that manages this manip.
      public: BuildingMaker *maker;

      /// \brief Visual's transparency.
      public: double transparency;

      /// \brief Visual's color.
      public: common::Color color;

      /// \brief Visual's texture.
      public: std::string texture;

      /// \brief Level this manipulator is on.
      public: int level;

      /// \brief A list of gui editor events connected to this view.
      public: std::vector<event::ConnectionPtr> connections;
    };
  }
}

#endif
