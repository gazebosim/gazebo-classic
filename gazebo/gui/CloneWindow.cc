/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/CloneWindow.hh"
#include "gazebo/gui/CloneWindowPrivate.hh"
#include "gazebo/gui/qt.h"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
CloneWindow::CloneWindow(QWidget *_parent)
  : QDialog(_parent),
    dataPtr(new CloneWindowPrivate())
{
  this->dataPtr->validPort = false;

  this->setWindowTitle(tr("Gazebo: Cloning a simulation"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // Create the main layout for this widget.
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QFrame *frame = new QFrame;

  // Port information.
  QLabel *portInfo = new QLabel(
    "Select the port that the new server will use for connections.\n"
    "Please check that the port is not used by any other process.\n");

  // Port widget.
  QHBoxLayout *portLayout = new QHBoxLayout;
  QLabel *portLabel = new QLabel("Cloned server port (1025-65535):");
  this->dataPtr->portEdit = new QLineEdit("11346");
  this->dataPtr->portEdit->setFixedWidth(50);
  this->dataPtr->portEdit->setMaxLength(5);
  this->dataPtr->portEdit->setValidator(new QIntValidator(1025, 65535, this));
  portLayout->setContentsMargins(4, 4, 4, 30);
  portLayout->addWidget(portLabel);
  portLayout->addWidget(this->dataPtr->portEdit);

  // Buttons.
  QHBoxLayout *buttonLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton("Cancel");
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));
  this->dataPtr->okayButton = new QPushButton("Ok");
  connect(this->dataPtr->okayButton, SIGNAL(clicked()), this, SLOT(OnOkay()));
  buttonLayout->addWidget(cancelButton);
  buttonLayout->addStretch(2);
  buttonLayout->addWidget(this->dataPtr->okayButton);

  // Compose the main frame.
  mainLayout->addWidget(portInfo);
  mainLayout->addWidget(frame);
  mainLayout->addLayout(portLayout);
  mainLayout->addLayout(buttonLayout);
  mainLayout->setContentsMargins(8, 8, 4, 4);

  // Assign the mainlayout to this widget.
  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
CloneWindow::~CloneWindow()
{
}

/////////////////////////////////////////////////
int CloneWindow::Port() const
{
  if (this->dataPtr->validPort)
    return this->dataPtr->port;
  else
    return 0;
}

/////////////////////////////////////////////////
bool CloneWindow::IsValidPort() const
{
  return this->dataPtr->validPort;
}

/////////////////////////////////////////////////
void CloneWindow::Update()
{
  this->dataPtr->port =
    this->dataPtr->portEdit->text().toInt(&this->dataPtr->validPort);
}

/////////////////////////////////////////////////
void CloneWindow::OnOkay()
{
  this->done(QDialog::Accepted);
  this->Update();
}

/////////////////////////////////////////////////
void CloneWindow::OnCancel()
{
  this->done(QDialog::Rejected);
}
