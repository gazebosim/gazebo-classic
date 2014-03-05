/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _CYLINDERMAKER_HH_
#define _CYLINDERMAKER_HH_

#include <string>

#include "gazebo/math/Vector2i.hh"
#include "gazebo/gui/EntityMaker.hh"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class CylinderMaker CylinderMaker.hh
    /// \brief to make a cylinder
    class CylinderMaker : public EntityMaker
    {
      /// \brief Constructor
      public: CylinderMaker();
      /// \brief Deconstructor
      public: virtual ~CylinderMaker();

      /// Documentation inherited
      public: virtual void Start(const rendering::UserCameraPtr _camera);
      /// Documentation inherited
      public: virtual void Stop();
      /// Documentation inherited
      public: virtual bool IsActive() const;

      /// Documentation inherited
      public: virtual void OnMousePush(const common::MouseEvent &_event);
      /// Documentation inherited
      public: virtual void OnMouseRelease(const common::MouseEvent &_event);
      /// Documentation inherited
      public: virtual void OnMouseDrag(const common::MouseEvent &_event);
      /// Documentation inherited
      public: virtual void OnMouseMove(const common::MouseEvent &_event);

      /// \brief Get the SDF information for the cylinder.
      /// \return The SDF as a string.
      public: std::string GetSDFString();

      /// Documentation inherited
      private: virtual void CreateTheEntity();
      private: int state;
      private: bool leftMousePressed;
      private: math::Vector2i mousePushPos, mouseReleasePos;
      private: msgs::Visual *visualMsg;

      private: static unsigned int counter;
    };
  }
}

#endif


