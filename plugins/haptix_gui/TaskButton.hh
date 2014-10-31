 /*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _GUI_TASK_BUTTON_HH_
#define _GUI_TASK_BUTTON_HH_

#include <QToolButton>
#include <QTextDocument>

namespace gazebo
{
  class TaskButton : public QToolButton
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _name Display name of the task.
    /// \param[in] _id Task id, used with the arrange plugin to setup the
    /// scene.
    /// \param[in] _taskIndex Number of the task.
    /// \param[in] _groupIndex Group this task belongs to.
    public: TaskButton(const std::string &_name, const std::string &_id,
                int _taskIndex, const int _groupIndex);

    /// \brief Set the task id (used in conjunction with the arrange plugin).
    /// \param[in] _id  Task ID
    public: void SetId(const std::string _id);

    /// \brief Get the task id (used in conjunction with the arrange plugin).
    /// \return Task ID
    public: std::string Id() const;

    /// \brief Set the task index.
    /// \param[in] _index Integer index.
    public: void SetIndex(const int _index);

    /// \brief Get the task index.
    /// \return Integer index.
    public: int Index() const;

    /// \brief Ste the task instructions.
    /// \param[in] _instr The instructions.
    public: void SetInstructions(const std::string &_instr);

    /// \brief Get the instructions.
    /// \return The task's instructions.
    public: QTextDocument *Instructions() const;

    /// \brief Callback when this button is pressed.
    public slots: void OnButton();

    /// \brief Get the task's group.
    /// \return Integer group.
    public: int Group() const;

    /// \brief Signal that sets the current task.
    /// \param[in] _id Task id.
    signals: void SendTask(const int _id);

    /// \brief Instructions for this task.
    private: QTextDocument *instructions;

    /// \brief The index at which this task is stored.
    private: int index;

    /// \brief Group this task belongs to.
    private: int group;

    /// \brief The id that is sent to the arrange plugin to organize the
    /// scene.
    private: std::string id;
  };
}
#endif
