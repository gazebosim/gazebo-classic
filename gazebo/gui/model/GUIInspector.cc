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

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/model/GUIInspector.hh"

#include "gazebo/rendering/Grid.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"

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

      /// \brief Message containing the data which was in the widget when first
      /// open.
      public: msgs::GUI originalDataMsg;

      /// \brief Pointer to the default grid in the scene.
      public: rendering::Grid *grid;

      /// \brief Pointer to the user camera.
      public: rendering::UserCameraPtr camera;
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
  msgs::GUI guiMsg;
  this->dataPtr->configWidget = new ConfigWidget;
  this->dataPtr->configWidget->Load(&guiMsg);

  // Hide unecessary fields
  this->dataPtr->configWidget->SetWidgetVisible("camera::name", false);
  this->dataPtr->configWidget->SetWidgetVisible("camera::view_controller",
      false);
  this->dataPtr->configWidget->SetWidgetVisible("camera::track", false);
  this->dataPtr->configWidget->SetWidgetVisible("camera::projection_type",
      false);
  this->dataPtr->configWidget->SetWidgetVisible("grid::name", false);

  // Custom projections widget
  std::vector<std::string> projs;
  projs.push_back("perspective");
  projs.push_back("orthographic");
  auto projWidget = this->dataPtr->configWidget->CreateEnumWidget(
      "Projection type", projs, 1);
  this->dataPtr->configWidget->AddConfigChildWidget("camera::projectionEnum",
      projWidget);

  auto camGroup = this->dataPtr->configWidget->GroupWidgetByName("camera");
  if (!camGroup)
  {
    gzerr << "Could not find camera group widget" << std::endl;
    return;
  }
  qobject_cast<QVBoxLayout *>((qobject_cast<QGroupBox *>(
      camGroup->childWidget->layout()->itemAt(0)->widget()))->layout())->
      addWidget(projWidget);

  // Connect all value changes
  this->connect(this->dataPtr->configWidget,
      SIGNAL(PoseValueChanged(const QString &,
      const ignition::math::Pose3d &)), this,
      SLOT(OnPoseChanged(const QString &, const ignition::math::Pose3d &)));

  this->connect(this->dataPtr->configWidget,
      SIGNAL(UIntValueChanged(const QString &, const unsigned int)), this,
      SLOT(OnUIntChanged(const QString &, const unsigned int)));

  this->connect(this->dataPtr->configWidget,
      SIGNAL(DoubleValueChanged(const QString &, const double)), this,
      SLOT(OnDoubleChanged(const QString &, const double)));

  this->connect(this->dataPtr->configWidget,
      SIGNAL(ColorValueChanged(const QString &, const gazebo::common::Color &)),
      this, SLOT(OnColorChanged(const QString &, const gazebo::common::Color &)));

  this->connect(this->dataPtr->configWidget,
      SIGNAL(EnumValueChanged(const QString &, const QString &)),
      this, SLOT(OnEnumChanged(const QString &, const QString &)));

  this->connect(this->dataPtr->configWidget,
      SIGNAL(BoolValueChanged(const QString &, const bool)),
      this, SLOT(OnBoolChanged(const QString &, const bool)));

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

  auto okButton = new QPushButton(tr("OK"));
  okButton->setEnabled(true);
  connect(okButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(resetButton);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(okButton);
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

  // Get rendering pointers
  this->dataPtr->camera = gui::get_active_camera();

  auto scene = this->dataPtr->camera->GetScene();
  this->dataPtr->grid = scene->GetGrid(0);
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
void GUIInspector::OnPoseChanged(const QString &_name,
    const ignition::math::Pose3d &_value)
{
  if (_name == "camera::pose" && this->dataPtr->camera)
    this->dataPtr->camera->SetWorldPose(_value);
}

/////////////////////////////////////////////////
void GUIInspector::OnUIntChanged(const QString &_name,
    const unsigned int _value)
{
  if (_name == "grid::cell_count" && this->dataPtr->grid)
    this->dataPtr->grid->SetCellCount(_value);
  else if (_name == "grid::normal_cell_count" && this->dataPtr->grid)
    this->dataPtr->grid->SetHeight(_value);
}

/////////////////////////////////////////////////
void GUIInspector::OnDoubleChanged(const QString &_name, const double _value)
{
  if (_name == "grid::cell_size" && this->dataPtr->grid)
    this->dataPtr->grid->SetCellLength(_value);
  else if (_name == "grid::height_offset" && this->dataPtr->grid)
    this->dataPtr->grid->SetHeightOffset(_value);
}

/////////////////////////////////////////////////
void GUIInspector::OnColorChanged(const QString &_name,
    const gazebo::common::Color &_value)
{
  if (_name == "grid::line_color" && this->dataPtr->grid)
    this->dataPtr->grid->SetColor(_value);
}

/////////////////////////////////////////////////
void GUIInspector::OnEnumChanged(const QString &_name, const QString &_value)
{
  if (_name == "camera::projectionEnum" && this->dataPtr->camera)
    this->dataPtr->camera->SetProjectionType(_value.toStdString());
}

/////////////////////////////////////////////////
void GUIInspector::OnBoolChanged(const QString &_name, const bool _value)
{
  if (_name == "fullscreen")
    gui::Events::fullScreen(_value);
}

/////////////////////////////////////////////////
void GUIInspector::Open()
{
  // Fill widgets
  this->dataPtr->configWidget->SetBoolWidgetValue("fulscreen",
      g_fullScreenAct->isChecked());

  // Camera
  this->dataPtr->configWidget->SetPoseWidgetValue("camera::pose",
      this->dataPtr->camera->WorldPose());
  this->dataPtr->configWidget->SetEnumWidgetValue("camera::projectionEnum",
      this->dataPtr->camera->ProjectionType());

  // Grid
  this->dataPtr->configWidget->SetUIntWidgetValue("grid::cell_count",
      this->dataPtr->grid->CellCount());
  this->dataPtr->configWidget->SetUIntWidgetValue("grid::normal_cell_count",
      this->dataPtr->grid->Height());
  this->dataPtr->configWidget->SetDoubleWidgetValue("grid::cell_size",
      this->dataPtr->grid->CellLength());
  this->dataPtr->configWidget->SetDoubleWidgetValue("grid::height_offset",
      this->dataPtr->grid->HeightOffset());
  this->dataPtr->configWidget->SetColorWidgetValue("grid::line_color",
      this->dataPtr->grid->Color());

  // Keep original data in case user cancels
  auto msg = dynamic_cast<msgs::GUI *>(
      this->dataPtr->configWidget->Msg());
  if (msg)
    this->dataPtr->originalDataMsg.CopyFrom(*msg);

  this->move(QCursor::pos());
  this->show();
}

/////////////////////////////////////////////////
void GUIInspector::RestoreOriginalData()
{
  msgs::GUIPtr msg;
  msg.reset(new msgs::GUI);
  msg->CopyFrom(this->dataPtr->originalDataMsg);

  // Update widgets
  this->dataPtr->configWidget->blockSignals(true);
  this->Update(msg);
  this->dataPtr->configWidget->blockSignals(false);
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
