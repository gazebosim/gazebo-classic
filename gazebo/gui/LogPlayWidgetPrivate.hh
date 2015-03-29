/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_LOG_PLAY_WIDGET_PRIVATE_HH_
#define _GAZEBO_LOG_PLAY_WIDGET_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class LogPlayWidgetPrivate LogPlayWidgetPrivate.hh
    /// \brief Private data for the LogPlayWidget class
    class LogPlayWidgetPrivate
    {
      /// \brief Display the simulation time.
      public: QLineEdit *simTimeEdit;

      /// \brief Display the number of iterations.
      public: QLineEdit *iterationsEdit;

      /// \brief Event based connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Mutex to protect the memeber variables.
      public: boost::mutex mutex;

      /// \brief Tool button that holds the step widget
      public: QToolButton *stepButton;

      /// \brief Sim time label.
      public: QLabel *simTimeLabel;

      /// \brief Iterations label.
      public: QLabel *iterationsLabel;

      /// \brief Action associated with the step label in the toolbar.
      public: QAction *stepToolBarLabelAction;

      /// \brief Action associated with the step button in the toolbar.
      public: QAction *stepButtonAction;

      /// \brief Paused state of the simulation.
      public: bool paused;

      /// \brief Paused state of the simulation.
      public: TimePanel *timePanel;
    };
  }
}
#endif
