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

#ifndef _GAZEBO_GUI_VARIABLE_PILL_CONTAINER_HH_
#define _GAZEBO_GUI_VARIABLE_PILL_CONTAINER_HH_

#include <memory>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    struct VariablePillContainerPrivate;

    class VariablePill;

    /// \brief A container for holding variable pills
    class GZ_GUI_VISIBLE VariablePillContainer : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to a parent widget
      public: VariablePillContainer(QWidget *_parent = NULL);

      /// \brief Destructor
      public: virtual ~VariablePillContainer();

      /// \brief Set the label text for this variable pill container.
      /// \param[in] _text Text to set the label to.
      public: void SetText(const std::string &_text);

      /// \brief Set the maximum number of variable pills this container can
      /// hold
      /// \param[in] _max Maximum number of variable pills. -1 means unlimited.
      public: void SetMaxSize(const int _max);

      /// \brief Get the maximum number of variable pills this container can
      /// hold
      /// \return Maximum number of variable pills. -1 means unlimited.
      public: int MaxSize() const;

      /// \brief Add a variable pill to the container.
      /// \param[in] _variable Variable pill to add.
      public: void AddVariablePill(VariablePill *_variable);

      /// \brief Remove a variable pill from the container.
      /// \param[in] _variable Variable pill to remove.
      public: void RemoveVariablePill(VariablePill *_variable);

      /// \brief Get the number of child variable pills
      /// \return Number of child variable pills.
      public: unsigned int VariablePillCount() const;

      /// \brief Used to accept drag enter events.
      /// \param[in] _evt The drag event.
      protected: void dragEnterEvent(QDragEnterEvent *_evt);

      /// \brief Used to accept drop events.
      /// \param[in] _evt The drop event.
      protected: void dropEvent(QDropEvent *_evt);

      /// \brief Qt signal emitted when a variable is added to the container
      /// \param[in] Unique id of the variable pill.
      /// \param[in] Name of variable pill added.
      Q_SIGNALS: void VariableAdded(const unsigned int ,
          const std::string &_name);

      /// \brief Qt signal emitted when a variable is removed from the container
      /// \param[in] Name of variable pill removed.
      Q_SIGNALS: void VariableRemoved(const unsigned int);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<VariablePillContainerPrivate> dataPtr;
    };
  }
}
#endif
