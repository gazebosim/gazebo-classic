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
#ifndef _MODEL_ALIGN_HH_
#define _MODEL_ALIGN_HH_

#include <string>

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class ModelAlignPrivate;

    /// \class ModelAlign ModelAlign.hh gui/Gui.hh
    /// \brief Manipulator tool for translating/rotating/scaling models and
    /// links
    class GAZEBO_VISIBLE ModelAlign : public SingletonT<ModelAlign>
    {
      /// \brief Constructor
      private: ModelAlign();

      /// \brief Destructor
      private: virtual ~ModelAlign();

      /// \brief Initialize the model manipulator.
      public: void Init();

      /// \brief Set the visual to be manipulated by the model manipulator.
      public: void SetAttachedVisual(rendering::VisualPtr _vis);

      /// \brief Process an object translate mouse press event.
      /// \param[in] _event Mouse event.
      public: void OnMousePressEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse move event.
      /// \param[in] _event Mouse event.
      public: void OnMouseMoveEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse release event.
      /// \param[in] _event Mouse event.
      public: void OnMouseReleaseEvent(const common::MouseEvent &_event);

      /// \brief Process a key press event.
      /// \param[in] _event Key event.
      public: void OnKeyPressEvent(const common::KeyEvent &_event);

      /// \brief Process a key release event.
      /// \param[in] _event Key event.
      public: void OnKeyReleaseEvent(const common::KeyEvent &_event);

      /// \brief Set the visual being moved by the mouse.
      /// \param[in] _vis Pointer to visual moved by mouse.
      private: void SetMouseMoveVisual(rendering::VisualPtr _vis);

      /// \brief This is a singleton class.
      private: friend class SingletonT<ModelAlign>;

      /// \internal
      /// \brief Pointer to private data.
      private: ModelAlignPrivate *dataPtr;
    };
  }
}
#endif
