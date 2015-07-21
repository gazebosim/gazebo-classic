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
#ifndef _GAZEBO_ENTITYMAKER_HH_
#define _GAZEBO_ENTITYMAKER_HH_

#include <ignition/math/Vector3.hh>

#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \ingroup gazebo_gui
  /// \brief gui namespace
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \brief to make an entity base class
    class GAZEBO_VISIBLE EntityMaker
    {
      /// \brief Constructor
      public: EntityMaker();

      /// \brief Destructor
      public: virtual ~EntityMaker();

      /// \brief Set whether to snap to grid
      public: static void SetSnapToGrid(bool _snap);

      /// \brief Start the maker.
      /// \param[in] _camera Pointer to the user camera
      public: virtual void Start();

      /// \brief Stop the maker.
      public: virtual void Stop();

      /// \brief Callback when mouse button is released
      /// \param[in] _event MouseEvent object
      public: virtual void OnMouseRelease(const common::MouseEvent &_event);

      /// \brief Callback when moving mouse
      /// \param[in] _event MouseEvent object
      public: virtual void OnMouseMove(const common::MouseEvent &_event);

      /// \brief Creates the entity
      protected: virtual void CreateTheEntity() = 0;

      /// \brief Returns the entity world position.
      /// \return Entity's position in the world frame.
      protected: virtual ignition::math::Vector3d EntityPosition() const;

      /// \brief Sets the entity world position.
      /// \param[in] _pos New position in the world frame.
      protected: virtual void SetEntityPosition(
          const ignition::math::Vector3d &_pos);

      /// \brief Node used to publish messages.
      protected: transport::NodePtr node;

      /// \brief Publisher for factory messages.
      protected: transport::PublisherPtr makerPub;

      /// \brief True to snap to grid while moving.
      private: static bool snapToGrid;
    };
  }
}
#endif


