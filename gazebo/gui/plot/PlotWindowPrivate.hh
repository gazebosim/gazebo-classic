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

#ifndef _GAZEBO_GUI_PLOTWINDOWPRIVATE_HH_
#define _GAZEBO_GUI_PLOTWINDOWPRIVATE_HH_


#include <mutex>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class IncrementalPlot;

    /// \brief Private data for the PlotWindow class
    class PlotWindowPrivate
    {
      /// \brief The list of diagnostic labels.
      public: QListWidget *labelList;

      /// \brief True when plotting is paused.
      public: bool paused = false;

      /// \brief Action to pause plotting
      public: QAction *plotPlayAct;

      /// \brief Action to resume plotting
      public: QAction *plotPauseAct;

      /// \brief Layout to hold all the canvases.
      public: QVBoxLayout *canvasLayout;

      /// \brief Mutex to protect the canvas updates
      public: std::mutex mutex;
    };
  }
}
#endif
