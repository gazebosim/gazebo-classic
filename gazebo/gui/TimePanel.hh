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
#ifndef _TIME_PANEL_HH_
#define _TIME_PANEL_HH_

#include <vector>
#include <list>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Time.hh"

class QLineEdit;
class QLabel;

namespace gazebo
{
  namespace gui
  {
    class TimePanel : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: TimePanel(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~TimePanel();

      /// \brief A signal used to set the sim time line edit.
      /// \param[in] _string String representation of sim time.
      signals: void SetSimTime(QString _string);

      /// \brief A signal used to set the real time line edit.
      /// \param[in] _string String representation of real time.
      signals: void SetRealTime(QString _string);

      /// \brief A signal used to set the iterations line edit.
      /// \param[in] _string String representation of iterations.
      signals: void SetIterations(QString _string);

      /// \brief Update the data output.
      private slots: void Update();

      /// \brief Called when the GUI enters/leaves full-screen mode.
      /// \param[in] _value True when entering full screen, false when
      /// leaving.
      private: void OnFullScreen(bool &_value);

      /// \brief Called when a world stats message is received.
      /// \param[in] _msg World statistics message.
      private: void OnStats(ConstWorldStatisticsPtr &_msg);

      /// \brief Helper function to format time string.
      /// \param[in] _msg Time message.
      private: static std::string FormatTime(const msgs::Time &_msg);

      /// \brief QT callback when the reset time button is pressed.
      private slots: void OnTimeReset();

      /// \brief Display the real time percentage.
      private: QLineEdit *percentRealTimeEdit;

      /// \brief Display the simulation time.
      private: QLineEdit *simTimeEdit;

      /// \brief Display the real time.
      private: QLineEdit *realTimeEdit;

      /// \brief Display the number of iterations.
      private: QLineEdit *iterationsEdit;

      /// \brief Node used for communication.
      private: transport::NodePtr node;

      /// \brief Subscriber to the statistics topic.
      private: transport::SubscriberPtr statsSub;

      /// \brief Used to start, stop, and step simulation.
      private: transport::PublisherPtr worldControlPub;

      /// \brief Event based connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief List of simulation times used to compute averages.
      private: std::list<common::Time> simTimes;

      /// \brief List of real times used to compute averages.
      private: std::list<common::Time> realTimes;

      /// \brief Mutex to protect the memeber variables.
      private: boost::mutex mutex;
    };
  }
}

#endif
