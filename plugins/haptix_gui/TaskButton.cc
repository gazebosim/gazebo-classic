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

#include "TaskButton.hh"

using namespace gazebo;

/////////////////////////////////////////////////
TaskButton::TaskButton(const std::string &_name, const std::string &_id,
    int _taskIndex, const int _groupIndex)
{
  this->id = _id;
  this->index = _taskIndex;
  this->group = _groupIndex;

  this->instructions = NULL;

  this->setCheckable(true);
  this->setText(QString::fromStdString(_name));
  this->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

  this->setStyleSheet(
      "QToolButton {"
        "color: rgba(128, 128, 128, 255);"
        "padding: 0px;"
        "border-radius: 4px;"
        "margin: 0px;"
      "}"

      "QToolButton:hover {"
        "background-color: #7A95D6;"
        "border: 0px;"
        "border-radius: 4px;"
        "color: #ffffff;"
      "}"

      "QToolButton:checked {"
        "background-color: rgba(83, 101, 146, 255);"
        "border: 0px;"
        "border-radius: 4px;"
        "color: #ffffff;"
      "}"

      );

  this->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  connect(this, SIGNAL(clicked()), this, SLOT(OnButton()));
}

/////////////////////////////////////////////////
void TaskButton::SetInstructions(const std::string &_instr)
{
  if (this->instructions)
    delete this->instructions;

  this->instructions =
    new QTextDocument(QString::fromStdString(_instr));
}

/////////////////////////////////////////////////
void TaskButton::SetId(const std::string _id)
{
  this->id = _id;
}

/////////////////////////////////////////////////
std::string TaskButton::Id() const
{
  return this->id;
}

/////////////////////////////////////////////////
void TaskButton::SetIndex(const int _index)
{
  this->index = _index;
}

/////////////////////////////////////////////////
int TaskButton::Index() const
{
  return this->index;
}

/////////////////////////////////////////////////
void TaskButton::OnButton()
{
  emit SendTask(this->index);
}

/////////////////////////////////////////////////
QTextDocument *TaskButton::Instructions() const
{
  return this->instructions;
}

/////////////////////////////////////////////////
int TaskButton::Group() const
{
  return this->group;
}
