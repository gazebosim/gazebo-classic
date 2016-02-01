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

      /// \brief Container size.
      public: int maxSize = -1;

      /// \brief Pointer to variable pill that is currently selected.
      public: VariablePill *selectedVariable = NULL;
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
  _variable->setVisible(true);
  this->dataPtr->variableLayout->addWidget(_variable);
  this->dataPtr->variables[_variable->Id()] = _variable;

  std::cerr << " emit variable added into container event " <<
      _variable->Text() << " " << _variable->Id() <<  std::endl;

  emit VariableAdded(_variable->Id(), VariablePill::EMPTY_ID,
      _variable->Text());
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
  _variable->setVisible(false);
  this->dataPtr->variableLayout->takeAt(idx);
  this->dataPtr->variables.erase(_variable->Id());

  emit VariableRemoved(_variable->Id(), VariablePill::EMPTY_ID);

  std::cerr
      << "VariablePillContainer emit variable removed from container event "
      << _variable->Text() << " " << _variable->Id() << std::endl;
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

/////////////////////////////////////////////////
void VariablePillContainer::SetSelected(VariablePill *_variable)
{
  if (this->dataPtr->selectedVariable)
    this->dataPtr->selectedVariable->SetSelected(false);

  this->dataPtr->selectedVariable = _variable;

  if (this->dataPtr->selectedVariable)
    this->dataPtr->selectedVariable->SetSelected(true);
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

    std::cerr << "variable '" << dataStr.toStdString()
        << "' dropped into container ["
        << this->dataPtr->label->text().toStdString() << "]"<< std::endl;

    VariablePill *variable = new VariablePill;
    variable->SetText(dataStr.toStdString());

    connect(variable, SIGNAL(VariableMoved(unsigned int)),
        this, SLOT(OnMoveVariable(unsigned int)));
    connect(variable, SIGNAL(VariableAdded(unsigned int, std::string)),
        this, SLOT(OnAddVariable(unsigned int, std::string)));
    connect(variable, SIGNAL(VariableRemoved(unsigned int)),
        this, SLOT(OnRemoveVariable(unsigned int)));

    this->AddVariablePill(variable);
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    VariablePill *variable = qobject_cast<VariablePill *>(_evt->source());
    if (!variable)
    {
      gzerr << "Variable is NULL" << std::endl;
      return;
    }

    // block signals and emit VariableMoved instead.
    VariablePill *parentVariable = variable->Parent();
    if (parentVariable)
    {
      std::cerr << " VariablePillContainer::dropEvent " <<
           variable->Parent()->Text() << std::endl;

      parentVariable->blockSignals(true);
      parentVariable->RemoveVariablePill(variable);
      parentVariable->blockSignals(false);
    }

    VariablePillContainer *container = variable->Container();
    if (container)
    {
      container->blockSignals(true);
      container->RemoveVariablePill(variable);
      container->blockSignals(false);
    }

    // case when the variable is dragged out from a muli-variable pill to
    // the container
    this->blockSignals(true);
    this->AddVariablePill(variable);
    this->blockSignals(false);

    emit VariableMoved(variable->Id(), VariablePill::EMPTY_ID);
    std::cerr << "VariablePillContainer emit variable moved event " <<
        variable->Text() << " " << variable->Id() << std::endl;
  }
}

/////////////////////////////////////////////////
void VariablePillContainer::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Delete)
  {
    if (this->dataPtr->selectedVariable)
    {
      this->RemoveVariablePill(this->dataPtr->selectedVariable);
    }
  }

}

/////////////////////////////////////////////////
void VariablePillContainer::mouseReleaseEvent(QMouseEvent *_event)
{
  this->SetSelected(NULL);

  bool selected = false;
  for (const auto v : this->dataPtr->variables)
  {
    // look for the selected variable widget if not already found
    QPoint point = v.second->mapFromParent(_event->pos());
    if (!selected)
    {
      ignition::math::Vector2i pt(point.x(), point.y());
      if (v.second->ContainsPoint(pt))
      {
        this->SetSelected(v.second);
        this->setFocus();
        selected = true;
      }
      else
      {
        v.second->SetSelected(false);
      }
    }
    else
    {
      v.second->SetSelected(false);
    }


    // loop through children of multi-variable pills
    for (const auto cv : v.second->VariablePills())
    {
      VariablePill *child = cv.second;

      if (!selected)
      {
        QPoint childPoint = child->mapFromParent(point);
        ignition::math::Vector2i childPt(childPoint.x(), childPoint.y());
        if (child->ContainsPoint(childPt))
        {
          this->SetSelected(child);
          this->setFocus();
          selected = true;
        }
        else
        {
          child->SetSelected(false);
        }
      }
      else
      {
        child->SetSelected(false);
      }
    }

  }
}

/////////////////////////////////////////////////
void VariablePillContainer::OnMoveVariable(const unsigned int _id)
{
  std::cerr << " VariablePillContainer::OnMoveVariable " << _id << std::endl;
  VariablePill *variable = qobject_cast<VariablePill *>(QObject::sender());
  if (!variable)
    return;

  emit VariableMoved(_id, variable->Id());
}

/////////////////////////////////////////////////
void VariablePillContainer::OnAddVariable(const unsigned int _id,
    const std::string &_label)
{
  VariablePill *variable = qobject_cast<VariablePill *>(QObject::sender());
  if (!variable)
    return;

  emit VariableAdded(_id, variable->Id(), _label);
}

/////////////////////////////////////////////////
void VariablePillContainer::OnRemoveVariable(const unsigned int _id)
{
  VariablePill *variable = qobject_cast<VariablePill *>(QObject::sender());
  if (!variable)
    return;

  emit VariableRemoved(_id, variable->Id());
}
