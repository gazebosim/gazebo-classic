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

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class MouseEvent;
  }

  /// \ingroup gazebo_gui
  /// \brief gui namespace
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class EntityMaker EntityMaker.hh gui/gui.hh
    /// \brief to make an entity base class
    class GAZEBO_VISIBLE EntityMaker
    {
      /// \def CreateCallback
      /// \brief boost::function with vector3 pos and vector3 scale
      public: typedef boost::function<void(const math::Vector3 &pos,
                  const math::Vector3 &scale)> CreateCallback;

      /// \brief Constructor
      public: EntityMaker();

      /// \brief Destructor
      public: virtual ~EntityMaker();

      /// \brief Set whether to snap to grid
      public: static void SetSnapToGrid(bool _snap);

      /// \brief Start the maker.
      /// \param[in] _camera Pointer to the user camera
      public: virtual void Start(const rendering::UserCameraPtr _camera) = 0;

      /// \brief Stop the maker.
      public: virtual void Stop() = 0;

      /// \brief Checks if maker is active
      /// \return True if is active,
      public: virtual bool IsActive() const = 0;

      /// \brief Callback when mouse button is released
      /// \param[in] _event MouseEvent object
      public: virtual void OnMouseRelease(const common::MouseEvent &_event);

      /// \brief Callback when moving mouse
      /// \param[in] _event MouseEvent object
      public: virtual void OnMouseMove(const common::MouseEvent &_event);

      /// \brief Get a point snapped to a 1m x 1m x 1m grid.
      /// \param[in] _p Point to be snapped
      /// \return Point on the grid
      protected: math::Vector3 GetSnappedPoint(math::Vector3 _p);

      /// \brief Creates the entity
      protected: virtual void CreateTheEntity() = 0;

      protected: rendering::UserCameraPtr camera;

      protected: transport::NodePtr node;
      protected: transport::PublisherPtr visPub;
      protected: transport::PublisherPtr makerPub;
      protected: transport::PublisherPtr requestPub;
      protected: CreateCallback createCB;


      private: static bool snapToGrid;
      private: static double snapDistance;
      private: static double snapGridSize;
    };
  }
}
#endif


