/*
 * Copyright 2011 Nate Koenig
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
#ifndef ENTITYMAKER_HH
#define ENTITYMAKER_HH

#include "rendering/RenderTypes.hh"
#include "transport/TransportTypes.hh"
#include "math/Vector3.hh"

namespace gazebo
{
  namespace common
  {
    class MouseEvent;
  }

  namespace gui
  {
    class EntityMaker
    {
      public: typedef boost::function<void(const math::Vector3 &pos,
                  const math::Vector3 &scale)> CreateCallback;

      /// \brief Constructor
      public: EntityMaker();

      /// \brief Destructor
      public: virtual ~EntityMaker();

      /// \brief Set whether to snap to grid
      public: static void SetSnapToGrid(bool _snap);

      public: virtual void Start(const rendering::UserCameraPtr _camera) = 0;

      public: virtual void Stop() = 0;
      public: virtual bool IsActive() const = 0;

      public: virtual void OnMousePush(const common::MouseEvent &_event);
      public: virtual void OnMouseRelease(const common::MouseEvent &_event);
      public: virtual void OnMouseDrag(const common::MouseEvent &_event);
      public: virtual void OnMouseMove(const common::MouseEvent &_event);

      // \brief Get a point snapped to a grid
      protected: math::Vector3 GetSnappedPoint(math::Vector3 _p);

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


