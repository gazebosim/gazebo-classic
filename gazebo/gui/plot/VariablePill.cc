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
      /// \brief Pointer to the main frame
      public: QFrame *mainFrame;

      /// \brief Pointer to the container this variable pill is in
      public: VariablePillContainer *container = nullptr;

      /// \brief Parent variable pill if this is inside a multi-variable pill.
      public: VariablePill *parent = nullptr;

      /// \brief Child variables pills if this is a multi-variable pill.
      public: std::map<unsigned int, VariablePill *> variables;

      /// \brief Layout that contains all the child variable pills.
      public: QHBoxLayout *variableLayout;

      /// \brief Text label
      public: QLabel *label;

      /// \brief Frame that holds the label
      public: QFrame *labelFrame;

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

      /// \brief Unique id.
      public: unsigned int id;

      /// \brief Global id incremented on every new variable pill
      public: static unsigned int globalVariableId;

      /// \brief unique name.
      public: std::string name;
    };
  }
}

// empty variable id
const unsigned int VariablePill::EmptyVariable = IGN_UINT32_MAX;

// global variable id counter
unsigned int VariablePillPrivate::globalVariableId = 0;

/////////////////////////////////////////////////
VariablePill::VariablePill(QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new VariablePillPrivate)
{
  this->dataPtr->id = VariablePillPrivate::globalVariableId;

  // generate default unique name
  std::stringstream nameStream;
  nameStream << "variable" << this->dataPtr->id << std::endl;
  this->dataPtr->name = nameStream.str();

  VariablePillPrivate::globalVariableId++;

  // label
  this->dataPtr->label = new QLabel;
  QHBoxLayout *labelLayout = new QHBoxLayout;
  labelLayout->addWidget(this->dataPtr->label);
  labelLayout->setContentsMargins(0, 0, 0, 0);
  this->dataPtr->labelFrame = new QFrame;
  this->dataPtr->labelFrame->setLayout(labelLayout);

  // child variable pills
  this->dataPtr->variableLayout = new QHBoxLayout;
  this->dataPtr->variableLayout->setAlignment(Qt::AlignLeft);

  this->dataPtr->multiLayout = new QHBoxLayout;
  this->dataPtr->multiLayout->setAlignment(Qt::AlignLeft);
  this->dataPtr->multiLayout->setContentsMargins(0, 0, 0, 0);
  this->dataPtr->multiLabel = new QLabel;
  this->dataPtr->multiLabel->setText(QString(" Variables:"));
  this->dataPtr->multiLabel->setVisible(false);
  this->dataPtr->multiLayout->addWidget(this->dataPtr->multiLabel);

  this->dataPtr->singleLayout = new QHBoxLayout;
  this->dataPtr->singleLayout->setAlignment(Qt::AlignLeft);
  this->dataPtr->singleLayout->addWidget(this->dataPtr->labelFrame);
  this->dataPtr->singleLayout->addLayout(this->dataPtr->variableLayout);
  this->dataPtr->singleLayout->setContentsMargins(0, 0, 0, 0);
  this->dataPtr->multiLayout->addLayout(this->dataPtr->singleLayout);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setAlignment(Qt::AlignLeft);

  this->dataPtr->mainFrame = new QFrame(this);
  this->dataPtr->mainFrame->setLayout(this->dataPtr->multiLayout);
  this->dataPtr->mainFrame->setObjectName("variablePillFrame");

  mainLayout->addWidget(this->dataPtr->mainFrame);
  this->setLayout(mainLayout);

  this->UpdateStyleSheet();
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
void VariablePill::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

/////////////////////////////////////////////////
std::string VariablePill::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
void VariablePill::SetText(const std::string &_text)
{
  QString text = QString::fromStdString(_text);
  this->dataPtr->label->setText(text);
  this->dataPtr->label->setToolTip(text);
  emit VariableLabelChanged(_text);
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

  int margin = _enable ? 4 : 0;
  this->dataPtr->multiLayout->setContentsMargins(
      margin, margin, margin, margin);

  this->UpdateStyleSheet();
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
  if (_variable->Parent() != nullptr && _variable->Parent() != _variable)
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
  _variable->UpdateStyleSheet();
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
      newMultiVariable->SetParent(nullptr);
      newMultiVariable->blockSignals(true);
      newMultiVariable->UpdateStyleSheet();
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
    // set parent and container to nullptr before calling
    // VariablePillContainer::RemoveVariablePill to prevent double removal
    VariablePillContainer *tmpContainer =  this->dataPtr->container;
    this->dataPtr->parent = nullptr;
    this->dataPtr->container = nullptr;
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
  _variable->setParent(nullptr);
  _variable->SetParent(nullptr);
  _variable->SetContainer(nullptr);
  _variable->UpdateStyleSheet();

  // becomes single variable pill
  if (this->dataPtr->variables.empty())
  {
    this->SetParent(nullptr);
    this->SetMultiVariableMode(false);
  }

  emit VariableRemoved(_variable->Id());
}

/////////////////////////////////////////////////
VariablePill *VariablePill::VariablePillByName(const std::string &_name)
{
  if (_name == this->dataPtr->name)
    return this;

  for (const auto &v : this->dataPtr->variables)
  {
    if (v.second->Name() == _name)
      return v.second;
  }

  return nullptr;
}

/////////////////////////////////////////////////
void VariablePill::dragEnterEvent(QDragEnterEvent *_evt)
{
  if (!this->IsDragValid(_evt))
  {
    _evt->setDropAction(Qt::IgnoreAction);
    _evt->accept();
    return;
  }

  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    _evt->setDropAction(Qt::LinkAction);
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    _evt->setDropAction(Qt::MoveAction);
  }
  else
  {
    _evt->ignore();
    return;
  }

  _evt->acceptProposedAction();
}

/////////////////////////////////////////////////
void VariablePill::dropEvent(QDropEvent *_evt)
{
  if (!this->IsDragValid(_evt))
  {
    _evt->accept();
    return;
  }

  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    QString mimeData = _evt->mimeData()->data("application/x-item");
    std::string dataStr = mimeData.toStdString();

    VariablePill *variable = new VariablePill;
    variable->SetText(dataStr);
    variable->SetName(dataStr);

    connect(variable, SIGNAL(VariableMoved(unsigned int)),
        this->Container(), SLOT(OnMoveVariable(unsigned int)));
    connect(variable, SIGNAL(VariableAdded(unsigned int, std::string)),
        this->Container(), SLOT(OnAddVariable(unsigned int, std::string)));
    connect(variable, SIGNAL(VariableRemoved(unsigned int)),
        this->Container(), SLOT(OnRemoveVariable(unsigned int)));
    connect(variable, SIGNAL(VariableLabelChanged(std::string)),
        this->Container(), SLOT(OnSetVariableLabel(std::string)));

    this->AddVariablePill(variable);
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    VariablePill *variable = qobject_cast<VariablePill *>(_evt->source());
    if (!variable)
    {
      gzerr << "Variable is nullptr" << std::endl;
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

    // add to parent if it exists, otherwise add to self and become a
    // multi-variable
    if (this->dataPtr->parent)
    {
      this->dataPtr->parent->blockSignals(true);
      this->dataPtr->parent->AddVariablePill(variable);
      this->dataPtr->parent->blockSignals(false);
    }
    else
    {
      this->blockSignals(true);
      this->AddVariablePill(variable);
      this->blockSignals(false);
    }

    emit VariableMoved(variable->Id());
  }
}
/////////////////////////////////////////////////
bool VariablePill::IsDragValid(QDropEvent *_evt)
{
  if (_evt->source() == this)
    return false;

  std::string variableName;
  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    QString mimeData = _evt->mimeData()->data("application/x-item");
    variableName = mimeData.toStdString();
  }
  else if (_evt->mimeData()->hasFormat("application/x-pill-item"))
  {
    VariablePill *dragVariable = qobject_cast<VariablePill *>(_evt->source());
    if (!dragVariable)
      return false;

    // limit drag and drop to same container
    if (dragVariable->Container() &&
        dragVariable->Container() != this->Container())
      return false;

    variableName = dragVariable->Name();
  }
  else
    return false;

  if (variableName.empty())
    return false;

  // check variable with same name is not in the variable pill already
  if (this->dataPtr->name == variableName ||
      (this->dataPtr->parent &&
      this->dataPtr->parent->VariablePillByName(variableName)))
  {
    return false;
  }

  // check max size
  if (this->Container() && this->Container()->MaxSize() != -1 &&
      static_cast<int>(this->Container()->VariablePillCount()) >=
      this->Container()->MaxSize())
  {
    return false;
  }

  return true;
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

  this->UpdateStyleSheet();
}

/////////////////////////////////////////////////
bool VariablePill::IsSelected() const
{
  return this->dataPtr->isSelected;
}

/////////////////////////////////////////////////
void VariablePill::UpdateStyleSheet()
{
  std::string colorHex;
  std::string bgColorStr;
  std::string borderStr;
  if (this->dataPtr->parent)
    colorHex = "#64b5f6";
  else
    colorHex = "#2196f3";

  bgColorStr = "background-color: " + colorHex + ";";

  if (this->dataPtr->isSelected)
    borderStr = "border: 1.5px solid #1565c0;";
  else
    borderStr = "border: 1.5px solid " + colorHex + ";";

  this->dataPtr->label->setStyleSheet(QString::fromStdString(
      "QLabel\
      {\
        color: #ffffff; \
        border-radius: 10px;\
        padding-left: 8px;\
        padding-right: 8px;\
        padding-top: 2px;\
        padding-bottom: 2px;\
        margin: 0px; "
        + bgColorStr + borderStr +
      "}"));
}
