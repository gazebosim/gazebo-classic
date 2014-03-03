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
#ifndef _TOPICVIEW_HH_
#define _TOPICVIEW_HH_

#include <string>
#include <list>
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Time.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \cond
    /// \brief A custom combobox that pull in a list of topics for user
    /// selection.
    class TopicCombo : public QComboBox
    {
      /// \brief Constructor
      /// \param[in] _w Parent widget.
      /// \param[in] _msgTypeName Name of the message type to display.
      /// \param[in] _viewType Type of viewer, used to prune list of topics.
      /// \param[in] _node Node used to get the list of topics.
      public: TopicCombo(QWidget *_w, const std::string &_msgTypeName,
                  const std::string &_viewType, transport::NodePtr _node);

      /// \brief Destructor
      public: virtual ~TopicCombo();

      /// \brief Set the message type name.
      /// \param[in] _type Name of the message type.
      public: void SetMsgTypeName(const std::string &_type);

      /// \brief Inherited function from QComboBox. Called when the item
      /// list is displayed.
      public: virtual void showPopup();

      /// \brief Update the list of items.
      private: void UpdateList();

      /// \brief Name of the message types to display.
      private: std::string msgTypeName;

      /// \brief Type of viewer.
      private: std::string viewType;

      /// \brief Tranport node pointer.
      private: transport::NodePtr node;

      private: boost::mutex mutex;
    };
    /// \endcond

    /// \brief Base class for widgets that display topic data.
    class TopicView : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      /// \param[in] _msgType Type of message that the viewer can display.
      /// \param[in] _parent Pointer to the parent widget.
      /// \param[in] _viewType The type of the viewer.
      /// \param[in] _displayPeriod Milliseconds between display updates.
      public: TopicView(QWidget *_parent, const std::string &_msgType,
                        const std::string &_viewType,
                        unsigned int _displayPeriod = 500);

      /// \brief Destructor
      public: virtual ~TopicView();

      /// \brief Set the name of the topic to get data from.
      /// \param[in] _topicName Name of the topic to use.
      public: virtual void SetTopic(const std::string &_topicName);

      /// \brief Used by child class to indicate when a message has been
      /// received.
      /// \param[in] _dataTime Time the data was created. This time should
      /// be the timestamp when data was generated on the server.
      /// \param[in] _size Size of the message in bytes.
      protected: void OnMsg(const common::Time &_dataTime, int _size);

      /// \brief Qt close event callback.
      /// \param[in] _event The close event info.
      protected: virtual void closeEvent(QCloseEvent *_event);

      /// \brief Update the camera sensor widget.
      private slots: void Update();

      /// \brief Update implementation. Each viewer must implement this to
      /// display the appropriate information.
      private: virtual void UpdateImpl();

      /// \brief QT callback triggered when the user has selected
      /// a different topic.
      /// \param[in] _index Index of the topic in the topic combobox.
      private slots: void OnTopicChanged(int _index);

      /// \brief This is the frame that each child class should populate.
      protected: QFrame *frame;

      /// \brief Pointer to the topic subscriber that receives display  data.
      protected: transport::SubscriberPtr sub;

      /// \brief Pointer to the node for communication.
      protected: transport::NodePtr node;

      /// \brief Typename of the messages that can be displayed.
      protected: std::string msgTypeName;

      /// \brief Combo box that displays all the relevant topics.
      private: TopicCombo *topicCombo;

      /// \brief Previous time a message was received.
      private: common::Time prevTime;

      /// \brief Previous display update time.
      private: common::Time prevDisplayTime;

      /// \brief Output for the hz info.
      private: QLineEdit *hzEdit;

      /// \brief The actual hz value.
      private: double hz;

      /// \brief Output for the bandwidth info.
      private: QLineEdit *bandwidthEdit;

      /// \brief A list of message sizes.
      private: std::list<int> msgSizes;

      /// \brief A list of clock times that messages have been received.
      private: std::list<common::Time> times;

      /// \brief A list of clock times that messages have been generated.
      private: std::list<common::Time> dataTimes;

      /// \brief A mutex to protect the update cycle.
      private: boost::mutex updateMutex;
    };
  }
}
#endif
