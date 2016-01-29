/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/model/CollisionConfig.hh"
#include "gazebo/gui/model/LinkConfig.hh"
#include "gazebo/gui/model/LinkInspector.hh"
#include "gazebo/gui/model/LinkInspectorPrivate.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/VisualConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LinkInspector::LinkInspector(QWidget *_parent) : QDialog(_parent),
    dataPtr(new LinkInspectorPrivate)
{
  this->setObjectName("LinkInspector");
  this->setWindowTitle(tr("Link Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QLabel *linkLabel = new QLabel(tr("Name:"));
  this->dataPtr->linkNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(linkLabel);
  nameLayout->addWidget(this->dataPtr->linkNameLabel, QSizePolicy::Maximum);
  nameLayout->setAlignment(this->dataPtr->linkNameLabel, Qt::AlignLeft);

  this->dataPtr->linkConfig = new LinkConfig;
  connect(this->dataPtr->linkConfig, SIGNAL(Applied()), this,
      SLOT(OnConfigApplied()));
  this->dataPtr->visualConfig = new VisualConfig;
  connect(this->dataPtr->visualConfig, SIGNAL(Applied()), this,
      SLOT(OnConfigApplied()));
  this->dataPtr->collisionConfig = new CollisionConfig;
  connect(this->dataPtr->collisionConfig, SIGNAL(Applied()), this,
      SLOT(OnConfigApplied()));

  // Create the main tab widget for all components in a link
  this->dataPtr->tabWidget = new QTabWidget();
  this->dataPtr->tabWidget->setObjectName("linkInspectorTab");
  this->dataPtr->tabWidget->setMinimumHeight(300);
  this->dataPtr->tabWidget->setMinimumWidth(560);

  this->dataPtr->tabWidget->addTab(this->dataPtr->linkConfig, "Link");
  this->dataPtr->tabWidget->addTab(this->dataPtr->visualConfig, "Visual");
  this->dataPtr->tabWidget->addTab(this->dataPtr->collisionConfig, "Collision");

  // Buttons
  QToolButton *removeButton = new QToolButton(this);
  removeButton->setFixedSize(QSize(30, 30));
  removeButton->setToolTip("Remove link");
  removeButton->setIcon(QPixmap(":/images/trashcan.png"));
  removeButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  removeButton->setIconSize(QSize(16, 16));
  removeButton->setCheckable(false);
  connect(removeButton, SIGNAL(clicked()), this, SLOT(OnRemove()));

  QPushButton *resetButton = new QPushButton(tr("Reset"));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(RestoreOriginalData()));

  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *OKButton = new QPushButton(tr("OK"));
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(removeButton);
  buttonsLayout->addStretch(5);
  buttonsLayout->addWidget(resetButton);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(nameLayout);
  mainLayout->addWidget(this->dataPtr->tabWidget);
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);

  // Conections
  connect(this, SIGNAL(rejected()), this, SLOT(RestoreOriginalData()));
}

/////////////////////////////////////////////////
LinkInspector::~LinkInspector()
{
}

/////////////////////////////////////////////////
void LinkInspector::SetName(const std::string &_name)
{
  this->dataPtr->linkNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
std::string LinkInspector::Name() const
{
  return this->dataPtr->linkNameLabel->text().toStdString();
}

/////////////////////////////////////////////////
LinkConfig *LinkInspector::GetLinkConfig() const
{
  return this->dataPtr->linkConfig;
}

/////////////////////////////////////////////////
VisualConfig *LinkInspector::GetVisualConfig() const
{
  return this->dataPtr->visualConfig;
}

/////////////////////////////////////////////////
CollisionConfig *LinkInspector::GetCollisionConfig() const
{
  return this->dataPtr->collisionConfig;
}

/////////////////////////////////////////////////
void LinkInspector::OnRemove()
{
  this->close();

  model::Events::requestLinkRemoval(this->dataPtr->linkId);
}

/////////////////////////////////////////////////
void LinkInspector::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void LinkInspector::OnConfigApplied()
{
  emit Applied();
}

/////////////////////////////////////////////////
void LinkInspector::OnOK()
{
  emit Accepted();
}

/////////////////////////////////////////////////
void LinkInspector::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}

/////////////////////////////////////////////////
void LinkInspector::SetLinkId(const std::string &_id)
{
  this->dataPtr->linkId = _id;
}

/////////////////////////////////////////////////
void LinkInspector::Open()
{
  this->dataPtr->linkConfig->Init();
  this->dataPtr->visualConfig->Init();
  this->dataPtr->collisionConfig->Init();

  this->move(QCursor::pos());
  this->show();
}

/////////////////////////////////////////////////
void LinkInspector::RestoreOriginalData()
{
  this->dataPtr->linkConfig->RestoreOriginalData();
  this->dataPtr->visualConfig->RestoreOriginalData();
  this->dataPtr->collisionConfig->RestoreOriginalData();

  emit Applied();
}

/////////////////////////////////////////////////
void LinkInspector::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Enter)
    _event->accept();
  else
    QDialog::keyPressEvent(_event);
}
