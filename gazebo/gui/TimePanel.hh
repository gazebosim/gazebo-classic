/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gui/qt.h"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Event.hh"
#include "common/Time.hh"

class QLineEdit;
class QLabel;

namespace gazebo
{
  namespace gui
  {
    class TimePanel : public QWidget
    {
      Q_OBJECT
      public: TimePanel(QWidget *_parent = 0);
      public: virtual ~TimePanel();

      private slots: void Update();

      private: void OnFullScreen(bool &_value);
      private: void OnStats(ConstWorldStatisticsPtr &_msg);

      private slots: void OnTimeReset();

      private: QLineEdit *percentRealTimeEdit;
      private: QLineEdit *simTimeEdit;
      private: QLineEdit *realTimeEdit;

      private: common::Time lastUpdateTime, statusUpdatePeriod;
      private: common::Time simTime, realTime, pauseTime;

      private: transport::NodePtr node;
      private: transport::SubscriberPtr statsSub;
      private: transport::PublisherPtr worldControlPub;

      private: std::vector<event::ConnectionPtr> connections;
      private: std::list<common::Time> simTimes, realTimes;
    };
  }
}

#endif
