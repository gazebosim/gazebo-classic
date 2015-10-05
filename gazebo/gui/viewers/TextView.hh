/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _TEXTVIEW_HH_
#define _TEXTVIEW_HH_

#include <string>
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Time.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/viewers/TopicView.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GZ_GUI_VIEWERS_VISIBLE TextView : public TopicView
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _msgType Type of message to view.
      public: TextView(QWidget *_parent, const std::string &_msgType);

      /// \brief Destructor
      public: virtual ~TextView();

      // Documentation inherited
      public: virtual void SetTopic(const std::string &_topicName);

      /// \brief Signal to add a message to the GUI list.
      /// \param[in] _msg Text message to add.
      signals: void AddMsg(QString _msg);

      // Documentation inherited
      private: virtual void UpdateImpl();

      /// \brief Receives incoming text messages.
      /// \param[in] _msg New text message.
      private: void OnText(const std::string &_msg);

      /// \brief QT callback when the buffer spin box has been changed.
      /// \param[in] _value New value of the spin box.
      private slots: void OnBuffer(int _value);

      /// \brief QT callback when the pause check box has been changed.
      /// \param[in] _value New value of the check box.
      private slots: void OnPause(bool _value);

      /// \brief Callback from the ::AddMsg function.
      /// \param[in] _msg Message to add to the list.
      private slots: void OnAddMsg(QString _msg);

      /// \brief A scolling list of text data.
      private: QListWidget *msgList;

      /// \brief Size of the text buffer. The size is the number of
      /// messages.
      private: int bufferSize;

      /// \brief The protobuf message used to decode incoming message data.
      private: boost::shared_ptr<google::protobuf::Message> msg;

      /// \brief Mutex to protect message buffer.
      private: boost::mutex mutex;

      /// \brief Flag used to pause message parsing.
      private: bool paused;
    };
  }
}
#endif
