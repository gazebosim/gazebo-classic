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

#include <iostream>
#include <map>

#include "gazebo/common/Console.hh"

#include "gazebo/gui/plot/VariablePill.hh"
#include "gazebo/gui/plot/VariablePillContainer.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief VariablePillContainer private data
    struct VariablePillContainerPrivate
    {
      /// \brief Text label
      public: QLabel *label;

      /// \brief Layout that contains all the variable pills.
      public: QLayout *variableLayout;

      /// \brief Variables inside this container
      public: std::map<unsigned int, VariablePill *> variables;

      /// \brief Container size
      public: int maxSize = -1;
    };
  }
}

/////////////////////////////////////////////////
VariablePillContainer::VariablePillContainer(QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new VariablePillContainerPrivate)
{
  // label
  this->dataPtr->label = new QLabel;
  QHBoxLayout *labelLayout = new QHBoxLayout;
  labelLayout->addWidget(this->dataPtr->label);

  // variable pills
  this->dataPtr->variableLayout = new QHBoxLayout;
  this->dataPtr->variableLayout->setAlignment(Qt::AlignLeft);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addLayout(labelLayout);
  mainLayout->addLayout(this->dataPtr->variableLayout);
  mainLayout->setAlignment(Qt::AlignLeft);
  this->setLayout(mainLayout);
  this->setAcceptDrops(true);
}

/////////////////////////////////////////////////
VariablePillContainer::~VariablePillContainer()
{
}

/////////////////////////////////////////////////
void VariablePillContainer::SetText(const std::string &_text)
{
  this->dataPtr->label->setText(QString::fromStdString(_text));
}

/////////////////////////////////////////////////
void VariablePillContainer::AddVariablePill(VariablePill *_variable)
{
  if (!_variable)
    return;

  if (this->dataPtr->variables.find(_variable->Id()) !=
      this->dataPtr->variables.end())
    return;

  _variable->SetContainer(this);
  this->dataPtr->variableLayout->addWidget(_variable);
  this->dataPtr->variables[_variable->Id()] = _variable;

  std::cerr << "add " << _variable->Text() <<
    " to container " << this->dataPtr->label->text().toStdString() << std::endl;

  emit VariableAdded(_variable->Id(), _variable->Text());
}

/////////////////////////////////////////////////
void VariablePillContainer::SetMaxSize(const int _max)
{
  this->dataPtr->maxSize = _max;
}

/////////////////////////////////////////////////
int VariablePillContainer::MaxSize() const
{
  return this->dataPtr->maxSize;
}

/////////////////////////////////////////////////
void VariablePillContainer::RemoveVariablePill(VariablePill *_variable)
{
  if (!_variable)
    return;

  if (this->dataPtr->variables.find(_variable->Id()) ==
      this->dataPtr->variables.end())
    return;

  int idx = this->dataPtr->variableLayout->indexOf(_variable);
  if (idx == -1)
    return;

  _variable->SetContainer(NULL);
  this->dataPtr->variableLayout->takeAt(idx);
  this->dataPtr->variables.erase(_variable->Id());
  std::cerr << "remove " << _variable->Text() <<
    " from container " << this->dataPtr->label->text().toStdString() << std::endl;

  emit VariableRemoved(_variable->Id());
}

/////////////////////////////////////////////////
void VariablePillContainer::dragEnterEvent(QDragEnterEvent *_evt)
{
  std::cerr << " this->VariablePillCount() "<< this->VariablePillCount()
      << std::endl;
  if (this->dataPtr->maxSize != -1 &&
      static_cast<int>(this->VariablePillCount()) >= this->dataPtr->maxSize)
  {
    _evt->ignore();
    return;
  }

  if (_evt->source() == this)
    return;

  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    _evt->setDropAction(Qt::LinkAction);
    _evt->acceptProposedAction();
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    _evt->setDropAction(Qt::MoveAction);
    _evt->acceptProposedAction();
  }
  else
    _evt->ignore();
}

/////////////////////////////////////////////////
void VariablePillContainer::dropEvent(QDropEvent *_evt)
{
  if (this->dataPtr->maxSize != -1 &&
      static_cast<int>(this->VariablePillCount()) >= this->dataPtr->maxSize)
  {
    std::cerr << "dropped and ignored "<< std::endl;
    _evt->ignore();
    return;
  }

  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    QString dataStr = _evt->mimeData()->data("application/x-item");

    // emit VariableDropped(dataStr.toStdString());

    std::cerr << "variable '" << dataStr.toStdString() << "' dropped into container [" <<
        this->dataPtr->label->text().toStdString() << "]"<< std::endl;

//    if (this->dataPtr->variables.find(dataStr.toStdString()) ==
//        this->dataPtr->variables.end())
    {
      VariablePill *variable = new VariablePill;
      variable->SetText(dataStr.toStdString());
      this->AddVariablePill(variable);
    }
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    VariablePill *variable = qobject_cast<VariablePill *>(_evt->source());
    if (!variable)
    {
      gzerr << "Variable is NULL" << std::endl;
      return;
    }

    if (variable->Parent())
    {
      std::cerr << " VariablePillContainer::dropEvent " <<
           variable->Parent()->Text() << std::endl;
      variable->Parent()->RemoveVariablePill(variable);
    }

    if (variable->Container())
      variable->Container()->RemoveVariablePill(variable);

    // case when the variable is dragged out from a muli-variable pill to
    // the container
    this->AddVariablePill(variable);
  }
}

/////////////////////////////////////////////////
unsigned int VariablePillContainer::VariablePillCount() const
{
  unsigned int count = 0;
  for (const auto v : this->dataPtr->variables)
  {
    count++;
    count += v.second->VariablePillCount();
  }
  return count;
}
