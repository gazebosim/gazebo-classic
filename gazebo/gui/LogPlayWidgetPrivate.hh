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

#include <string>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class LogPlayWidgetPrivate LogPlayWidgetPrivate.hh
    /// \brief Private data for the LogPlayWidget class
    class LogPlayWidgetPrivate
    {
      /// \brief Node used for communication.
      public: transport::NodePtr node;

      /// \brief Subscriber to the statistics topic.
      public: transport::SubscriberPtr statsSub;

      /// \brief Event based connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief List of simulation times used to compute averages.
      public: std::list<common::Time> simTimes;

      /// \brief Mutex to protect the memeber variables.
      public: boost::mutex mutex;

      /// TODO
      public: LogPlayView *view;
    };

    /// \class LogPlayViewPrivate LogPlayViewPrivate.hh
    /// \brief Private data for the LogPlayView class
    class LogPlayViewPrivate
    {
      /// \brief Node used for communication.
      public: transport::NodePtr node;

      /// \brief TODO
      public: QGraphicsPolygonItem *playItem;

      /// \brief TODO
      public: QGraphicsSimpleTextItem *playText;

      /// \brief TODO
      public: QGraphicsSimpleTextItem *stepText;

      /// \brief TODO
      public: QGraphicsSimpleTextItem *timeText;

      /// \brief TODO
      public: QGraphicsSimpleTextItem *itText;

      /// \brief Used to start, stop, and step simulation.
      public: transport::PublisherPtr worldControlPub;
    };
  }
}
#endif
