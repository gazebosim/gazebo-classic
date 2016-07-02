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

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/GUIInspector.hh"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the GUIInspector class
    class GUIInspectorPrivate
    {
      /// \brief Config widget for configuring joint properties.
      public: ConfigWidget *configWidget;

      /// \brief Ok button.
      public: QPushButton *okButton;

      /// \brief A list of gui editor events connected to this.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief Message containing the data which was in the widget when first
      /// open.
      public: msgs::GUI originalDataMsg;
    };
  }
}

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GUIInspector::GUIInspector(QWidget *_parent)
    : QDialog(_parent), dataPtr(new GUIInspectorPrivate)
{
  this->setObjectName("GUIInspectorDialog");
  this->setWindowTitle(tr("GUI Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // Config widget
  this->dataPtr->configWidget = new ConfigWidget;

  // Camera
  auto poseWidget = this->dataPtr->configWidget->CreatePoseWidget("Pose", 0);
  this->dataPtr->configWidget->AddConfigChildWidget("camera::pose",
      poseWidget);

  auto camWidget =
      this->dataPtr->configWidget->CreateGroupWidget("Camera", poseWidget, 0);

  // Grid


  // Config layout
  auto configLayout = new QVBoxLayout();
  configLayout->addWidget(camWidget);
  this->dataPtr->configWidget->setLayout(configLayout);

  // Connect all value changes
  connect(this->dataPtr->configWidget, SIGNAL(PoseValueChanged(const QString &,
      const ignition::math::Pose3d &)), this,
      SLOT(OnPoseChanged(const QString &, const ignition::math::Pose3d &)));

  // Connect vector value changes, for axes
  connect(this->dataPtr->configWidget,
      SIGNAL(Vector3dValueChanged(const QString &,
      const ignition::math::Vector3d &)), this,
      SLOT(OnVector3dChanged(const QString &,
      const ignition::math::Vector3d &)));

  // Scroll area
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(this->dataPtr->configWidget);
  scrollArea->setWidgetResizable(true);

  // General layout
  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);

  // Buttons
  QPushButton *resetButton = new QPushButton(tr("Reset"));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(RestoreOriginalData()));

  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  this->dataPtr->okButton = new QPushButton(tr("OK"));
  this->dataPtr->okButton->setEnabled(true);
  connect(this->dataPtr->okButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(resetButton);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(this->dataPtr->okButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setMinimumWidth(500);
  this->setMinimumHeight(300);

  this->setLayout(mainLayout);

  // Qt signal / slot connections
  connect(this, SIGNAL(rejected()), this, SLOT(RestoreOriginalData()));
}

/////////////////////////////////////////////////
GUIInspector::~GUIInspector()
{
}

/////////////////////////////////////////////////
void GUIInspector::Update(ConstGUIPtr _guiMsg)
{
  this->dataPtr->configWidget->UpdateFromMsg(_guiMsg.get());
}

/////////////////////////////////////////////////
void GUIInspector::OnPoseChanged(const QString &/*_name*/,
    const ignition::math::Pose3d &/*_pose*/)
{
}

/////////////////////////////////////////////////
void GUIInspector::OnVector3dChanged(const QString &/*_name*/,
    const ignition::math::Vector3d &/*_vec*/)
{
}

/////////////////////////////////////////////////
void GUIInspector::Open()
{
  // Fill widgets

  // Keep original data in case user cancels
  auto msg = dynamic_cast<msgs::GUI *>(
      this->dataPtr->configWidget->Msg());
  if (msg)
    this->dataPtr->originalDataMsg.CopyFrom(*msg);

  // Make sure the dialog opens with the proper fields showing
  this->blockSignals(true);
  this->RestoreOriginalData();
  this->blockSignals(false);

  this->move(QCursor::pos());
  this->show();
}

/////////////////////////////////////////////////
void GUIInspector::RestoreOriginalData()
{
  msgs::GUIPtr guiPtr;
  guiPtr.reset(new msgs::GUI);
  guiPtr->CopyFrom(this->dataPtr->originalDataMsg);

  // Update default widgets
  this->dataPtr->configWidget->blockSignals(true);
  this->Update(guiPtr);

  // Update custom widgets
}

/////////////////////////////////////////////////
void GUIInspector::OnCancel()
{
  this->RestoreOriginalData();

  this->reject();
}

/////////////////////////////////////////////////
void GUIInspector::OnOK()
{
  this->accept();
}

/////////////////////////////////////////////////
void GUIInspector::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}

/////////////////////////////////////////////////
void GUIInspector::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Enter)
    _event->accept();
  else
    QDialog::keyPressEvent(_event);
}

///////////////////////////////////////////////////
void GUIInspector::closeEvent(QCloseEvent *_event)
{
  _event->accept();
}
