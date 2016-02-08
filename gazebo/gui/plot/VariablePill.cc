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

#include "gazebo/common/Console.hh"

#include "gazebo/gui/plot/VariablePillContainer.hh"
#include "gazebo/gui/plot/VariablePill.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief VariablePill private data
    class VariablePillPrivate
    {
      /// \brief Pointer to the container this variable pill is in
      public: VariablePillContainer *container = NULL;

      /// \brief Parent variable pill if this is inside a multi-variable pill.
      public: VariablePill *parent = NULL;

      /// \brief Child variables pills if this is a multi-variable pill.
      public: std::map<unsigned int, VariablePill *> variables;

      /// \brief Layout that contains all the child variable pills.
      public: QHBoxLayout *variableLayout;

      /// \brief Text label
      public: QLabel *label;

      /// \brief Text label for the outer mulit-variable pill
      public: QLabel *multiLabel;

      /// \brief Layout for the single ariable pill
      public: QHBoxLayout *singleLayout;

      /// \brief Layout for a mulit-variable pill
      public: QHBoxLayout *multiLayout;

      /// \brief Starting position of the drag action.
      public: QPoint dragStartPosition;

      /// \brief Selected state.
      public: bool isSelected = false;

      /// \brief Unique id;
      public: unsigned int id;

      /// \brief Global id incremented on every new variable pill
      public: static unsigned int globalVariableId;
    };
  }
}

// empty variable id
unsigned int VariablePill::EMPTY_VARIABLE = IGN_UINT32_MAX;

// global variable id counter
unsigned int VariablePillPrivate::globalVariableId = 0;

/////////////////////////////////////////////////
VariablePill::VariablePill(QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new VariablePillPrivate)
{
  this->dataPtr->id = VariablePillPrivate::globalVariableId++;

  // label
  this->dataPtr->label = new QLabel;
  QHBoxLayout *labelLayout = new QHBoxLayout;
  labelLayout->addWidget(this->dataPtr->label);

  // child variable pills
  this->dataPtr->variableLayout = new QHBoxLayout;
  this->dataPtr->variableLayout->setAlignment(Qt::AlignLeft);
  this->dataPtr->variableLayout->setContentsMargins(0, 0, 0, 0);

  this->dataPtr->multiLayout = new QHBoxLayout;
  this->dataPtr->multiLayout->setAlignment(Qt::AlignLeft);
  this->dataPtr->multiLayout->setContentsMargins(0, 0, 0, 0);
  this->dataPtr->multiLabel = new QLabel;
  this->dataPtr->multiLabel->setText(QString("Variables:"));
  this->dataPtr->multiLabel->setVisible(false);
  this->dataPtr->multiLayout->addWidget(this->dataPtr->multiLabel);

  this->dataPtr->singleLayout = new QHBoxLayout;
  this->dataPtr->singleLayout->setAlignment(Qt::AlignLeft);
  this->dataPtr->singleLayout->addLayout(labelLayout);
  this->dataPtr->singleLayout->addLayout(this->dataPtr->variableLayout);
  this->dataPtr->singleLayout->setContentsMargins(0, 0, 0, 0);
  this->dataPtr->multiLayout->addLayout(this->dataPtr->singleLayout);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setAlignment(Qt::AlignLeft);

  QFrame *mainFrame = new QFrame(this);
  mainFrame->setLayout(this->dataPtr->multiLayout);

  mainFrame->setStyleSheet("background-color: #1965b0; border-radius: 4px");

  mainLayout->addWidget(mainFrame);
  this->setLayout(mainLayout);

  this->SetSelected(false);
  this->setAcceptDrops(true);
}

/////////////////////////////////////////////////
VariablePill::~VariablePill()
{
}

/////////////////////////////////////////////////
unsigned int VariablePill::Id() const
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
void VariablePill::SetText(const std::string &_text)
{
  this->dataPtr->label->setText(QString::fromStdString(_text));
}

/////////////////////////////////////////////////
std::string VariablePill::Text() const
{
  return this->dataPtr->label->text().toStdString();
}

/////////////////////////////////////////////////
void VariablePill::SetParent(VariablePill *_parent)
{
  this->dataPtr->parent = _parent;
}

/////////////////////////////////////////////////
VariablePill *VariablePill::Parent() const
{
  return this->dataPtr->parent;
}

/////////////////////////////////////////////////
void VariablePill::SetContainer(VariablePillContainer *_container)
{
  this->dataPtr->container = _container;
}

/////////////////////////////////////////////////
VariablePillContainer *VariablePill::Container() const
{
  return this->dataPtr->container;
}

/////////////////////////////////////////////////
void VariablePill::SetMultiVariableMode(const bool _enable)
{
  this->dataPtr->multiLabel->setVisible(_enable);

  int margin = _enable ? 2 : 0;
  this->dataPtr->multiLayout->setContentsMargins(
      margin, margin, margin, margin);
}

/////////////////////////////////////////////////
void VariablePill::AddVariablePill(VariablePill *_variable)
{
  if (!_variable)
    return;

  // check container capacity
  if (this->Container() && this->Container()->MaxSize() != -1 &&
    static_cast<int>(this->Container()->VariablePillCount()) >=
    this->Container()->MaxSize())
  {
    return;
  }

  if (this->dataPtr->parent && this->dataPtr->parent != this)
  {
    // Cannot add a variable pill to one that already has a parent.
    // Add to the parent instead
    this->dataPtr->parent->AddVariablePill(_variable);
    return;
  }

  // remove variable from parent if it already has one
  if (_variable->Parent() != NULL && _variable->Parent() != _variable)
    _variable->Parent()->RemoveVariablePill(_variable);

  if (this->dataPtr->variables.empty())
  {
    // becomes multi-variable pill
    this->SetParent(this);
    // enable multi-variable mode
    this->SetMultiVariableMode(true);
  }

  _variable->SetParent(this);
  _variable->setVisible(true);
  _variable->SetContainer(this->dataPtr->container);
  this->dataPtr->variables[_variable->Id()] = _variable;
  this->dataPtr->variableLayout->addWidget(_variable);

  emit VariableAdded(_variable->Id(), _variable->Text());
}

/////////////////////////////////////////////////
void VariablePill::RemoveVariablePill(VariablePill *_variable)
{
  // case for removing itself from multi-variable pill
  if (_variable == this)
  {
    if (!this->dataPtr->variables.empty())
    {
      // make first child variable a multi variable and move all the children
      QLayoutItem *item = this->dataPtr->variableLayout->takeAt(0);
      VariablePill *newMultiVariable =
          qobject_cast<VariablePill *>(item->widget());
      newMultiVariable->SetParent(NULL);
      newMultiVariable->blockSignals(true);
      while (this->dataPtr->variableLayout->count() > 0)
      {
        QLayoutItem *it = this->dataPtr->variableLayout->takeAt(0);
        VariablePill *var = qobject_cast<VariablePill *>(it->widget());

        newMultiVariable->AddVariablePill(var);
      }
      newMultiVariable->blockSignals(false);
      this->dataPtr->container->blockSignals(true);
      this->dataPtr->container->AddVariablePill(newMultiVariable);
      this->dataPtr->container->blockSignals(false);
    }
    // set parent and container to NULL before calling
    // VariablePillContainer::RemoveVariablePill to prevent double removal
    VariablePillContainer *tmpContainer =  this->dataPtr->container;
    this->dataPtr->parent = NULL;
    this->dataPtr->container = NULL;
    this->dataPtr->variables.clear();
    tmpContainer->blockSignals(true);
    tmpContainer->RemoveVariablePill(this);
    tmpContainer->blockSignals(false);
    this->SetMultiVariableMode(false);

    emit VariableRemoved(_variable->Id());

    return;
  }

  // remove a child variable
  int idx = this->dataPtr->variableLayout->indexOf(_variable);
  if (idx == -1)
    return;

  this->dataPtr->variableLayout->takeAt(idx);
  this->dataPtr->variables.erase(_variable->Id());
  _variable->setVisible(false);
  _variable->setParent(NULL);
  _variable->SetParent(NULL);
  _variable->SetContainer(NULL);

  // becomes single variable pill
  if (this->dataPtr->variables.empty())
  {
    this->SetParent(NULL);
    this->SetMultiVariableMode(false);
  }

  emit VariableRemoved(_variable->Id());
}

/////////////////////////////////////////////////
void VariablePill::dragEnterEvent(QDragEnterEvent *_evt)
{
  if (_evt->source() == this)
  {
    _evt->ignore();
    return;
  }

  if (this->Container() && this->Container()->MaxSize() != -1 &&
    static_cast<int>(this->Container()->VariablePillCount()) >=
    this->Container()->MaxSize())
  {
    _evt->ignore();
    return;
  }

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
void VariablePill::dropEvent(QDropEvent *_evt)
{
  if (_evt->source() == this)
  {
    _evt->ignore();
    return;
  }

  if (this->Container() && this->Container()->MaxSize() != -1 &&
    static_cast<int>(this->Container()->VariablePillCount()) >=
    this->Container()->MaxSize())
  {
    _evt->ignore();
    return;
  }

  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    QString dataStr = _evt->mimeData()->data("application/x-item");
    VariablePill *variable = new VariablePill;
    variable->SetText(dataStr.toStdString());

    connect(variable, SIGNAL(VariableMoved(unsigned int)),
        this->Container(), SLOT(OnMoveVariable(unsigned int)));
    connect(variable, SIGNAL(VariableAdded(unsigned int, std::string)),
        this->Container(), SLOT(OnAddVariable(unsigned int, std::string)));
    connect(variable, SIGNAL(VariableRemoved(unsigned int)),
        this->Container(), SLOT(OnRemoveVariable(unsigned int)));

    this->AddVariablePill(variable);
  }
  else   if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    VariablePill *variable = qobject_cast<VariablePill *>(_evt->source());
    if (!variable)
    {
      gzerr << "Variable is NULL" << std::endl;
      return;
    }

    VariablePill *parentVariable = variable->Parent();
    if (parentVariable)
    {
      parentVariable->blockSignals(true);
      parentVariable->RemoveVariablePill(variable);
      parentVariable->blockSignals(false);
    }
    else
    {
      VariablePillContainer *container = variable->Container();
      if (container)
      {
        container->blockSignals(true);
        container->RemoveVariablePill(variable);
        container->blockSignals(false);
      }
    }

    this->blockSignals(true);
    this->AddVariablePill(variable);
    this->blockSignals(false);

    emit VariableMoved(variable->Id());
  }
}

/////////////////////////////////////////////////
void VariablePill::mousePressEvent(QMouseEvent *_event)
{
  if (_event->button() == Qt::LeftButton)
    this->dataPtr->dragStartPosition = _event->pos();
}

/////////////////////////////////////////////////
void VariablePill::mouseMoveEvent(QMouseEvent *_event)
{
  if (!(_event->buttons() & Qt::LeftButton))
      return;

  if ((_event->pos() - this->dataPtr->dragStartPosition).manhattanLength()
       < QApplication::startDragDistance())
      return;

  QLabel *child = static_cast<QLabel *>(
      this->childAt(this->dataPtr->dragStartPosition));

  // prevent dragging by the multi-variable label
  if (child == this->dataPtr->multiLabel)
    return;

  QDrag *drag = new QDrag(this);
  QMimeData *mimeData = new QMimeData;
  QString textData = this->dataPtr->label->text();
  mimeData->setData("application/x-pill-item", textData.toLocal8Bit());
  mimeData->setText(textData);
  drag->setMimeData(mimeData);

  drag->exec(Qt::MoveAction);
}

/////////////////////////////////////////////////
bool VariablePill::ContainsPoint(const ignition::math::Vector2i &_pt) const
{
  QLabel *child = static_cast<QLabel *>(
    this->childAt(_pt.X(), _pt.Y()));

  if (child && child == this->dataPtr->label)
    return true;

  return false;
}

/////////////////////////////////////////////////
unsigned int VariablePill::VariablePillCount() const
{
  return this->dataPtr->variables.size();
}

/////////////////////////////////////////////////
std::map<unsigned int, VariablePill *> &VariablePill::VariablePills() const
{
  return this->dataPtr->variables;
}

/////////////////////////////////////////////////
void VariablePill::SetSelected(const bool _selected)
{
  this->dataPtr->isSelected = _selected;

  if (_selected)
  {
    this->dataPtr->label->setStyleSheet(
        "background-color: #5289c7; border: 2px solid black");
  }
  else
  {
    this->dataPtr->label->setStyleSheet(
        "background-color: #5289c7; border: 0px");
  }
}

/////////////////////////////////////////////////
bool VariablePill::IsSelected() const
{
  return this->dataPtr->isSelected;
}
