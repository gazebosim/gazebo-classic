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

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ViewAngleWidgetPrivate.hh"
#include "gazebo/gui/ViewAngleWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ViewAngleWidget::ViewAngleWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new ViewAngleWidgetPrivate)
{
  // Keep pointer to main window
  this->dataPtr->mainWindow = qobject_cast<MainWindow *>(_parent);

  // Lateral buttons
  this->dataPtr->topButton = new QToolButton(this);
  this->dataPtr->bottomButton = new QToolButton(this);
  this->dataPtr->frontButton = new QToolButton(this);
  this->dataPtr->backButton = new QToolButton(this);
  this->dataPtr->leftButton = new QToolButton(this);
  this->dataPtr->rightButton = new QToolButton(this);

  // Reset / home button
  this->dataPtr->resetButton = new QToolButton(this);

  // Button size
  QSize iconSize(30, 30);
  this->dataPtr->topButton->setIconSize(iconSize);
  this->dataPtr->bottomButton->setIconSize(iconSize);
  this->dataPtr->frontButton->setIconSize(iconSize);
  this->dataPtr->backButton->setIconSize(iconSize);
  this->dataPtr->leftButton->setIconSize(iconSize);
  this->dataPtr->rightButton->setIconSize(iconSize);
  this->dataPtr->resetButton->setIconSize(iconSize);

  // Dropdown
  this->dataPtr->projectionComboBox = new QComboBox(this);
  this->dataPtr->projectionComboBox->setMinimumWidth(150);
  this->dataPtr->projectionComboBox->addItem("Perspective", 0);
  this->dataPtr->projectionComboBox->addItem("Orthographic", 1);
  connect(this->dataPtr->projectionComboBox, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnProjection(int)));

  // Main layout
  this->dataPtr->mainLayout = new QGridLayout();
  this->dataPtr->mainLayout->addWidget(this->dataPtr->projectionComboBox,
      3, 0, 1, 4);
  this->setLayout(this->dataPtr->mainLayout);

  // Connect the ortho action
  connect(g_cameraOrthoAct, SIGNAL(triggered()), this, SLOT(OnOrtho()));

  // connect the perspective action
  connect(g_cameraPerspectiveAct, SIGNAL(triggered()), this,
      SLOT(OnPerspective()));
}

/////////////////////////////////////////////////
ViewAngleWidget::~ViewAngleWidget()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ViewAngleWidget::Add(const Mode _mode, QAction *_action)
{
  if (_mode == TOP)
  {
    this->dataPtr->topButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->topButton, 0, 1);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnTopView()));
  }
  else if (_mode == BOTTOM)
  {
    this->dataPtr->bottomButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->bottomButton, 2, 1);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnBottomView()));
  }
  else if (_mode == FRONT)
  {
    this->dataPtr->frontButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->frontButton, 1, 1);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnFrontView()));
  }
  else if (_mode == BACK)
  {
    this->dataPtr->backButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->backButton, 1, 3);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnBackView()));
  }
  else if (_mode == LEFT)
  {
    this->dataPtr->leftButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->leftButton, 1, 0);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnLeftView()));
  }
  else if (_mode == RIGHT)
  {
    this->dataPtr->rightButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->rightButton, 1, 2);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnRightView()));
  }
  else if (_mode == RESET)
  {
    this->dataPtr->resetButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->resetButton, 0, 3);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnResetView()));
  }
}

/////////////////////////////////////////////////
void ViewAngleWidget::LookDirection(const ignition::math::Vector3d &_dir)
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (!cam)
    return;

  GLWidget *glWidget =
      this->dataPtr->mainWindow->findChild<GLWidget *>("GLWidget");
  if (!glWidget)
  {
    gzerr << "GLWidget not found, this should never happen. " <<
              "Camera pose won't be changed." << std::endl;
    return;
  }

  // Look at world origin unless there are visuals selected
  ignition::math::Vector3d lookAt = ignition::math::Vector3d::Zero;

  // If there are selected visuals, look at their center
  std::vector<rendering::VisualPtr> selectedVisuals =
      glWidget->SelectedVisuals();

  if (!selectedVisuals.empty())
  {
    for (auto const &vis : selectedVisuals)
    {
      ignition::math::Vector3d visPos = vis->GetWorldPose().pos.Ign();
      lookAt += visPos;
    }
    lookAt /= selectedVisuals.size();
  }

  // Keep current distance to look target
  ignition::math::Vector3d camPos = cam->WorldPose().Pos();
  double distance = std::fabs((camPos - lookAt).Length());

  // Calculate camera position
  ignition::math::Vector3d newCamPos = lookAt - _dir * distance;

  // Calculate camera orientation
  double roll = 0;
  double pitch = -std::atan2(_dir.Z(),
      std::sqrt(std::pow(_dir.X(), 2) + std::pow(_dir.Y(), 2)));
  double yaw = std::atan2(_dir.Y(), _dir.X());

  ignition::math::Quaterniond quat =
      ignition::math::Quaterniond(roll, pitch, yaw);

  // Move camera to that pose in 1s
  cam->MoveToPosition(math::Pose(math::Vector3(newCamPos),
      math::Quaternion(quat)), 1);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnTopView()
{
  this->LookDirection(-ignition::math::Vector3d::UnitZ);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnBottomView()
{
  this->LookDirection(ignition::math::Vector3d::UnitZ);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnFrontView()
{
  this->LookDirection(-ignition::math::Vector3d::UnitX);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnBackView()
{
  this->LookDirection(ignition::math::Vector3d::UnitX);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnLeftView()
{
  this->LookDirection(-ignition::math::Vector3d::UnitY);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnRightView()
{
  this->LookDirection(ignition::math::Vector3d::UnitY);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnResetView()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();

  if (!cam)
    return;

  cam->MoveToPosition(cam->DefaultPose(), 1);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnPerspective()
{
  this->dataPtr->projectionComboBox->blockSignals(true);
  this->dataPtr->projectionComboBox->setCurrentIndex(0);
  this->dataPtr->projectionComboBox->blockSignals(false);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnOrtho()
{
  this->dataPtr->projectionComboBox->blockSignals(true);
  this->dataPtr->projectionComboBox->setCurrentIndex(1);
  this->dataPtr->projectionComboBox->blockSignals(false);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnProjection(int _index)
{
  if (_index == 0)
    g_cameraPerspectiveAct->trigger();
  else if (_index == 1)
    g_cameraOrthoAct->trigger();
  else
    gzwarn << "Projection index [" << _index << "] not recognized" << std::endl;
}

