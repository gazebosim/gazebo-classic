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

#ifndef _TOPIC_PLOT_HH_
#define _TOPIC_PLOT_HH_

#include <string>
#include <boost/thread/mutex.hpp>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class IncrementalPlot;

    /// \brief Class to plot data from a topic
    class GAZEBO_VISIBLE TopicPlot : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      public: TopicPlot(QWidget *_parent);

      /// \brief Destructor
      public: virtual ~TopicPlot();

      /// \brief Setup to plot a specific topic.
      public: void Init(const std::string &_topic, const std::string &_field);

      /// \brief Set the period over which to plot.
      /// \param[in] _seconds Period duration in seconds.
      public: void SetPeriod(unsigned int _seconds);

      /// \brief Called when a message is recieved.
      /// \param[in] _msg Message data.
      private: void OnMsg(const std::string &_msg);

      /// \brief Called when a clock message is recieved.
      /// \param[in] _msg CLock data.
      private: void OnClock(ConstTimePtr &_msg);

      /// \brief QT callback to redraw the plot.
      private slots: void Update();

      /// \brief Plotting widget.
      private: IncrementalPlot *plot;

      /// \brief Node for communications.
      private: transport::NodePtr node;

      /// \brief Subscribes to diagnostic info.
      private: transport::SubscriberPtr msgSub;

      /// \brief Subscribes to clock.
      private: transport::SubscriberPtr clockSub;

      /// \brief Typename of the messages that can be displayed.
      protected: std::string msgTypeName;

      /// \brief The protobuf message used to decode incoming message data.
      private: boost::shared_ptr<google::protobuf::Message> msg;

      /// \brief Layout to hold all the plots.
      private: QVBoxLayout *plotLayout;

      /// \brief Current simulation time.
      private: common::Time time;

      /// \brief Mutex to protect message buffer.
      private: boost::mutex mutex;

      /// \brief Topic name
      private: std::string topic;

      /// \brief Field name
      private: std::vector<std::string> fields;
    };
  }
}
#endif
