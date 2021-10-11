/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "gazebo/gui/RenderOptionsWindow.hh"
#include "gazebo/gui/RenderOptionsWindowPrivate.hh"
#include "gazebo/gui/qt.h"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RenderOptionsWindow::RenderOptionsWindow(QWidget *_parent)
  : QDialog(_parent),
    dataPtr(new RenderOptionsWindowPrivate())
{
  this->setWindowTitle(tr("Gazebo: Render Options"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // Create the main layout for this widget.
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QFrame *frame = new QFrame;

  // Port widget.
  QHBoxLayout *renderRateLayout = new QHBoxLayout;
  QLabel *RenderRateLabel = new QLabel("Render rate:");
  this->dataPtr->renderRateEdit = new QLineEdit("62.0");
  this->dataPtr->renderRateEdit->setFixedWidth(50);
  this->dataPtr->renderRateEdit->setMaxLength(5);
  this->dataPtr->renderRateEdit->setValidator(new QDoubleValidator(1.0, 512.0,
      5, this));
  renderRateLayout->setContentsMargins(4, 4, 4, 30);
  renderRateLayout->addWidget(RenderRateLabel);
  renderRateLayout->addWidget(this->dataPtr->renderRateEdit);

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
  mainLayout->addWidget(frame);
  mainLayout->addLayout(renderRateLayout);
  mainLayout->addLayout(buttonLayout);
  mainLayout->setContentsMargins(8, 8, 4, 4);

  // Assign the mainlayout to this widget.
  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
RenderOptionsWindow::~RenderOptionsWindow()
{
}

/////////////////////////////////////////////////
float RenderOptionsWindow::RenderRate() const
{
  return this->dataPtr->renderRate;
}

/////////////////////////////////////////////////
void RenderOptionsWindow::Update()
{
  this->dataPtr->renderRate =
    this->dataPtr->renderRateEdit->text().toFloat();
}

/////////////////////////////////////////////////
void RenderOptionsWindow::OnOkay()
{
  this->done(QDialog::Accepted);
  this->Update();
}

/////////////////////////////////////////////////
void RenderOptionsWindow::OnCancel()
{
  this->done(QDialog::Rejected);
}
