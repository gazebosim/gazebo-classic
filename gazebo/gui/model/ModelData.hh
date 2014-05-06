/*
 * Copyright (C) 2013-2014 Open Source Robotics Foundation
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
#ifndef _MODEL_DATA_HH_
#define _MODEL_DATA_HH_

#include <string>
#include <vector>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/physics/PhysicsTypes.hh"

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class CollisionData CollisionData.hh
    /// \brief Helper class to store collision data
    class CollisionData
    {
      /// \brief Name of collision.
      public: std::string name;

      /// \brief Pose of collision.
      public: math::Vector3 pose;

      /// \brief Sensor topic name.
      public: std::string geometry;
    };

    /// \class PartData PartData.hh
    /// \brief Helper class to store part data
    class PartData : public QObject
    {
      Q_OBJECT

      /// \brief Name of part.
      public: std::string name;

      /// \brief True to enable gravity on part.
      public: bool gravity;

      /// \brief True to allow self collision.
      public: bool selfCollide;

      /// \brief True to make part kinematic.
      public: bool kinematic;

      /// \brief Pose of part.
      public: math::Pose pose;

      /// \brief Inertial properties.
      public: physics::InertialPtr inertial;

      /// \brief Visual representing this part.
      public: rendering::VisualPtr partVisual;

      /// \brief Visuals of the part.
      public: std::vector<rendering::VisualPtr> visuals;

      /// \brief Collisions of the part.
      public: std::vector<CollisionData *> collisions;
    };
  }
}

#endif
