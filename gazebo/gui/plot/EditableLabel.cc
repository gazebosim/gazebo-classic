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
#include "gazebo/gui/plot/EditableLabel.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief EditableLabel private data
    class EditableLabelPrivate
    {
      /// \brief Widget for editing the label
      public: QLineEdit *lineEdit;

      /// \brief Label to be displayed
      public: QLabel *label;
    };
  }
}

/////////////////////////////////////////////////
EditableLabel::EditableLabel(const std::string &_label, QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new EditableLabelPrivate())
{
  this->dataPtr->lineEdit = new QLineEdit(this);
  connect(this->dataPtr->lineEdit, SIGNAL(editingFinished()),
    this, SLOT(OnEditingFinished()));
  this->dataPtr->lineEdit->hide();
  this->dataPtr->label = new QLabel(QString::fromStdString(_label), this);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(this->dataPtr->label);
  mainLayout->addWidget(this->dataPtr->lineEdit);
  this->setLayout(mainLayout);

  this->ShowBorder(false);

  this->installEventFilter(this);
}

/////////////////////////////////////////////////
EditableLabel::~EditableLabel()
{
}

/////////////////////////////////////////////////
void EditableLabel::ShowBorder(const bool _show)
{
  QString borderColor;
  if (_show)
    borderColor = "black";
  else
    borderColor = "transparent";

  this->dataPtr->label->setStyleSheet(
      "QLabel\
      {\
        border-radius: 6px;\
        border: 1px solid " + borderColor +";\
      }");
}

/////////////////////////////////////////////////
void EditableLabel::mouseDoubleClickEvent(QMouseEvent */*_event*/)
{
  this->dataPtr->label->hide();
  this->dataPtr->lineEdit->setText(this->dataPtr->label->text());
  this->dataPtr->lineEdit->show();
  this->dataPtr->lineEdit->setFocus();
  this->dataPtr->lineEdit->selectAll();
}

/////////////////////////////////////////////////
bool EditableLabel::eventFilter(QObject *_object, QEvent *_event)
{
  if (_object != this)
    return false;

  if (_event->type() == QEvent::Enter)
  {
    this->ShowBorder(true);
    return true;
  }
  else if (_event->type() == QEvent::Leave)
  {
    this->ShowBorder(false);
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
void EditableLabel::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Escape)
  {
    this->dataPtr->lineEdit->hide();
    this->dataPtr->label->show();
  }
}

/////////////////////////////////////////////////
void EditableLabel::OnEditingFinished()
{
  if (!this->dataPtr->lineEdit->isVisible())
    return;
  this->dataPtr->lineEdit->hide();
  this->dataPtr->label->setText(this->dataPtr->lineEdit->text());
  this->dataPtr->label->show();
}
