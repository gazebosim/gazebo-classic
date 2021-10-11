/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef _GAZEBO_CLONE_WINDOW_PRIVATE_HH_
#define _GAZEBO_CLONE_WINDOW_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class RenderOptionsWindowPrivate RenderOptionsWindowPrivate.hh
    /// \brief Private data for the RenderOptionsWindowPrivate class.
    class RenderOptionsWindowPrivate
    {
      /// \brief Button used to finalize render options.
      public: QPushButton *okayButton;

      /// \brief QT widget for reading the render rate used in the cloned 
      ///     server.
      public: QLineEdit *renderRateEdit;

      /// \brief Render rate used.
      public: float renderRate;
    };
  }
}
#endif
