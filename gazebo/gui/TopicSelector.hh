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
#ifndef _TOPICSELECTOR_HH_
#define _TOPICSELECTOR_HH_

#include <string>

#include "gazebo/common/Time.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class TopicSelector TopicSelector.hh gui/TopicSelector.hh
    /// \brief A widget that provides a list of topics to select from.
    class GZ_GUI_VISIBLE TopicSelector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget pointer.
      public: TopicSelector(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~TopicSelector();

      /// \brief Get the topic that was selected.
      /// \return The selected topic, which can be an empty string if no
      /// topic was selected.
      public: std::string GetTopic() const;

      /// \brief Get the message type that was selected.
      /// \return The selected topic's published message type.
      public: std::string GetMsgType() const;

      /// \brief Utility function to get the list of topic names.
      private: void GetTopicList();

      /// \brief Callback when okay button is selected.
      private slots: void OnOkay();

      /// \brief Callback when cancel button is selected.
      private slots: void OnCancel();

      /// \brief Callback when an item is selected with a single mouse click.
      /// \param[in] _item Pointer to the item selected.
      /// \param[in] _column Column selected.
      private slots: void OnSelection(QTreeWidgetItem *_item, int _column);

      /// \brief Callback when an item is selected with a double mouse click.
      /// \param[in] _item Pointer to the item selected.
      /// \param[in] _column Column selected.
      private slots: void OnDoubleClickSelection(QTreeWidgetItem *_item,
                                                 int _column);

      /// \brief The tree widget which holds all the topics.
      private: QTreeWidget *treeWidget;

      /// \brief Button used to finalize topic selection
      private: QPushButton *okayButton;

      /// \brief The name of the selected topic.
      private: std::string topicName;

      /// \brief The message type of the selected topic.
      private: std::string msgType;

      /// \brief Previous update time.
      private: common::Time prevTime;
    };
    /// \}
  }
}
#endif
