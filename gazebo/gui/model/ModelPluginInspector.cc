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

#include "gazebo/common/Console.hh"

#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelPluginInspector.hh"
#include "gazebo/gui/model/ModelPluginInspectorPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelPluginInspector::ModelPluginInspector(QWidget *_parent)
  : QDialog(_parent), dataPtr(new ModelPluginInspectorPrivate)
{
  this->setObjectName("ModelPluginInspector");
  this->setWindowTitle(tr("Model Plugin Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);
  this->setModal(false);

  this->setMinimumWidth(500);
  this->setMinimumHeight(300);

  // Config widget
  msgs::Plugin pluginMsg;
  this->dataPtr->configWidget = new ConfigWidget;
  this->dataPtr->configWidget->Load(&pluginMsg);

  this->SetReadOnly(true);

  // set a more user friendly label for name
  QWidget *childWidget =
      this->dataPtr->configWidget->ConfigChildWidgetByName("name");
  if (childWidget)
  {
    QList<QLabel *> labels = childWidget->findChildren<QLabel *>();
    for (auto &label : labels)
    {
      std::string labelLower = label->text().toLower().toStdString();
      if (labelLower == "name")
      {
        label->setText(QString("Plugin name"));
        break;
      }
    }
  }
  else
    gzerr << "Cannot find name field in model plugin inspector" << std::endl;

  // Scroll area
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(this->dataPtr->configWidget);
  scrollArea->setWidgetResizable(true);

  // General layout
  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);

  // Buttons
  QToolButton *removeButton = new QToolButton(this);
  removeButton->setFixedSize(QSize(30, 30));
  removeButton->setToolTip("Remove model plugin");
  removeButton->setIcon(QPixmap(":/images/trashcan.png"));
  removeButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  removeButton->setIconSize(QSize(16, 16));
  removeButton->setCheckable(false);
  connect(removeButton, SIGNAL(clicked()), this, SLOT(OnRemove()));

  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *OKButton = new QPushButton(tr("OK"));
  OKButton->setDefault(true);
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(removeButton);
  buttonsLayout->addStretch(5);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
ModelPluginInspector::~ModelPluginInspector()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnRemove()
{
  std::string pluginName =
      this->dataPtr->configWidget->StringWidgetValue("name");

  this->OnCancel();

  // User request from inspector
  model::Events::requestModelPluginRemoval(pluginName, true);
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnCancel()
{
  this->Clear();
  this->close();
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnOK()
{
  emit Applied();
  this->Clear();
  this->close();
}

/////////////////////////////////////////////////
void ModelPluginInspector::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}

/////////////////////////////////////////////////
void ModelPluginInspector::Update(ConstPluginPtr _pluginMsg)
{
  this->dataPtr->configWidget->UpdateFromMsg(_pluginMsg.get());
}

/////////////////////////////////////////////////
void ModelPluginInspector::SetReadOnly(const bool _readOnly)
{
  this->dataPtr->readOnly = _readOnly;
  this->dataPtr->configWidget->SetWidgetReadOnly("name", _readOnly);
  this->dataPtr->configWidget->SetWidgetReadOnly("filename", _readOnly);
  this->dataPtr->configWidget->SetWidgetReadOnly("innerxml", _readOnly);
}

/////////////////////////////////////////////////
void ModelPluginInspector::Clear()
{
  if (this->dataPtr->readOnly)
    return;
  this->dataPtr->configWidget->SetStringWidgetValue("name", "");
  this->dataPtr->configWidget->SetStringWidgetValue("filename", "");
  this->dataPtr->configWidget->SetStringWidgetValue("innerxml", "");
}

/////////////////////////////////////////////////
msgs::Plugin *ModelPluginInspector::Data() const
{
  msgs::Plugin *msg = dynamic_cast<msgs::Plugin *>(
      this->dataPtr->configWidget->Msg());
  if (!msg)
  {
    gzerr << "It wasn't possible to get the plugin message" << std::endl;
    return NULL;
  }
  return msg;
}
