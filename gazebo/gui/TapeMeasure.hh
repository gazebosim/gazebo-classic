/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_TAPEMEASURE_HH_
#define _GAZEBO_GUI_TAPEMEASURE_HH_

#include <memory>
#include <string>
#include <vector>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/math/Pose.hh"
#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class TapeMeasurePrivate;

    /// \class TapeMeasure TapeMeasure.hh gui/Gui.hh
    /// \brief A gui tool for snapping one model to another.
    class GZ_GUI_VISIBLE TapeMeasure : public SingletonT<TapeMeasure>
    {
      /// \brief Constructor
      private: TapeMeasure();

      /// \brief Destructor
      private: virtual ~TapeMeasure();

      /// \brief Initialize the model snapping tool.
      public: void Init();

      /// \brief Clear the model snapping tool. This explicity cleans up the
      /// internal state of the singleton and prepares it for exit.
      public: void Clear();

      /// \brief Reset the model snapping tool.
      public: void Reset();

      /// \brief Process an object translate mouse press event.
      /// \param[in] _event Mouse event.
      public: void OnMousePressEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse move event.
      /// \param[in] _event Mouse event.
      public: void OnMouseMoveEvent(const common::MouseEvent &_event);

      /// \brief Process an object translate mouse release event.
      /// \param[in] _event Mouse event.
      public: void OnMouseReleaseEvent(const common::MouseEvent &_event);

      /// \brief Update the visual representation of the snap spot.
      private: void Update();

      /// \brief This is a singleton class.
      private: friend class SingletonT<TapeMeasure>;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<TapeMeasurePrivate> dataPtr;
    };
  }
}
#endif
