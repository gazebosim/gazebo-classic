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
#ifndef _GAZEBO_TIME_WIDGET_PRIVATE_HH_
#define _GAZEBO_TIME_WIDGET_PRIVATE_HH_

#include <vector>
#include <list>
#include <mutex>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the TimeWidget class
    class TimeWidgetPrivate
    {
      /// \brief Sim time label.
      public: QLabel *simTimeLabel;

      /// \brief Display the simulation time.
      public: QLineEdit *simTimeEdit;

      /// \brief Real time label.
      public: QLabel *realTimeLabel;

      /// \brief Display the real time.
      public: QLineEdit *realTimeEdit;

      /// \brief Real time factor label.
      public: QLabel *realTimeFactorLabel;

      /// \brief Display the real time percentage.
      public: QLineEdit *percentRealTimeEdit;

      /// \brief Iterations label.
      public: QLabel *iterationsLabel;

      /// \brief Display the number of iterations.
      public: QLineEdit *iterationsEdit;

      /// \brief FPS label.
      public: QLabel *fpsLabel;

      /// \brief Display the average frames per second.
      public: QLineEdit *fpsEdit;

      /// \brief Action associated with the step label in the toolbar.
      public: QAction *stepToolBarLabelAction;

      /// \brief Tool button that holds the step widget
      public: QToolButton *stepButton;

      /// \brief Action associated with the step button in the toolbar.
      public: QAction *stepButtonAction;

      /// \brief Paused state of the simulation.
      public: bool paused;

      /// \brief Event based connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Node used for communication.
      public: transport::NodePtr node;

      /// \brief Subscriber to the statistics topic.
      public: transport::SubscriberPtr statsSub;

      /// \brief Used to start, stop, and step simulation.
      public: transport::PublisherPtr worldControlPub;

      /// \brief Mutex to protect the memeber variables.
      public: std::mutex mutex;

      /// \brief List of simulation times used to compute averages.
      public: std::list<common::Time> simTimes;

      /// \brief List of real times used to compute averages.
      public: std::list<common::Time> realTimes;
    };
  }
}
#endif
