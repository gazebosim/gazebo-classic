/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/ModelPluginInspector.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelPluginInspector::ModelPluginInspector(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("ModelPluginInspector");
  this->setWindowTitle(tr("Model Plugin Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // Config widget
  msgs::Plugin pluginMsg;
  this->configWidget = new ConfigWidget;
  this->configWidget->Load(&pluginMsg);

  this->configWidget->SetWidgetReadOnly("name", true);
  this->configWidget->SetWidgetReadOnly("filename", true);
  this->configWidget->SetWidgetReadOnly("innerxml", true);

  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(this->configWidget);
  scrollArea->setWidgetResizable(true);

  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);

  // Buttons
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));
  QPushButton *OKButton = new QPushButton(tr("OK"));
  OKButton->setDefault(true);
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);

  this->setMinimumWidth(500);
  this->setMinimumHeight(300);
}

/////////////////////////////////////////////////
ModelPluginInspector::~ModelPluginInspector()
{
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnOK()
{
  emit Accepted();
}

/////////////////////////////////////////////////
void ModelPluginInspector::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}

/////////////////////////////////////////////////
void ModelPluginInspector::Update(ConstPluginPtr _pluginMsg)
{
  this->configWidget->UpdateFromMsg(_pluginMsg.get());
}

