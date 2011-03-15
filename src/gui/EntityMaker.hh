/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "common/Vector3.hh"

namespace gazebo
{
	namespace gui
  {
    class Scene;
    class MouseEvent;
  
    class EntityMaker
    {
      /// \brief Constructor
      public: EntityMaker();
  
      /// \brief Destructor
      public: virtual ~EntityMaker();
  
      /// \brief Set whether to snap to grid
      public: static void SetSnapToGrid(bool snap);
  
      public: virtual void Start(Scene *scene) = 0;
      public: virtual void Stop() = 0;
      public: virtual bool IsActive() const = 0;
  
      public: virtual void MousePushCB(const MouseEvent &event);
      public: virtual void MouseReleaseCB(const MouseEvent &event);
      public: virtual void MouseDragCB(const MouseEvent &event);
  
      // \brief Get a point snapped to a grid
      protected: common::Vector3 GetSnappedPoint(common::Vector3 p);
  
      protected: virtual void CreateTheEntity() = 0;
  
      private: static bool snapToGrid;
      private: static double snapDistance;
      private: static double snapGridSize;
    };
  }
}
#endif
