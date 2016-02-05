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
#ifndef _GAZEBO_GUI_PLOTWINDOW_HH_
#define _GAZEBO_GUI_PLOTWINDOW_HH_

#include <memory>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotWindowPrivate;

    /// \brief Plot window
    class GZ_GUI_VISIBLE PlotWindow : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Pointer to the parent widget.
      public: PlotWindow(QWidget *_parent = NULL);

      /// \brief Destructor.
      public: virtual ~PlotWindow();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotWindowPrivate> dataPtr;
    };
  }
}
#endif
