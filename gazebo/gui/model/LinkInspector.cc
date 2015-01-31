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

#include "gazebo/common/Console.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/ConfigWidget.hh"


#include "gazebo/gui/model/LinkConfig.hh"
#include "gazebo/gui/model/VisualConfig.hh"
#include "gazebo/gui/model/CollisionConfig.hh"
#include "gazebo/gui/model/LinkInspector.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LinkInspector::LinkInspector(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("LinkInspector");
  this->setWindowTitle(tr("Link Inspector"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  QLabel *partLabel = new QLabel(tr("Name:"));
  this->partNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(partLabel);
  nameLayout->addWidget(partNameLabel);

  this->linkConfig = new LinkConfig;
  this->visualConfig = new VisualConfig;
  this->collisionConfig = new CollisionConfig;

  // Create the main tab widget for all components in a part
  this->tabWidget = new QTabWidget();
  this->tabWidget->setObjectName("LinkInspectorTab");
  this->tabWidget->setMinimumHeight(800);

  this->tabWidget->addTab(this->linkConfig, "Link");
  this->tabWidget->addTab(this->visualConfig, "Visual");
  this->tabWidget->addTab(this->collisionConfig, "Collision");

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));
  QPushButton *applyButton = new QPushButton(tr("&Apply"));
  connect(applyButton, SIGNAL(clicked()), this, SLOT(OnApply()));
  QPushButton *OKButton = new QPushButton(tr("&OK"));
  OKButton->setDefault(true);
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(applyButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(nameLayout);
  mainLayout->addWidget(tabWidget);
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
LinkInspector::~LinkInspector()
{
  delete this->linkConfig;
  this->linkConfig = NULL;
  delete this->visualConfig;
  this->visualConfig = NULL;
  delete this->collisionConfig;
  this->collisionConfig = NULL;
}

/////////////////////////////////////////////////
void LinkInspector::SetName(const std::string &_name)
{
  this->partNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
std::string LinkInspector::GetName() const
{
  return this->partNameLabel->text().toStdString();
}

/////////////////////////////////////////////////
LinkConfig *LinkInspector::GetLinkConfig() const
{
  return this->linkConfig;
}

/////////////////////////////////////////////////
VisualConfig *LinkInspector::GetVisualConfig() const
{
  return this->visualConfig;
}

/////////////////////////////////////////////////
CollisionConfig *LinkInspector::GetCollisionConfig() const
{
  return this->collisionConfig;
}

/////////////////////////////////////////////////
void LinkInspector::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void LinkInspector::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void LinkInspector::OnOK()
{
  emit Applied();
  this->accept();
}
