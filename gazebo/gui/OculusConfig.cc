/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/gui/OculusConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
OculusConfig::OculusConfig(QWidget *_parent)
  : QDialog(_parent)
{
  // This name is used in the qt style sheet
  this->setObjectName("oculusconfig");
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Oculus Rift configuration"));

  // Create the main layout for this widget.
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QFrame *frame = new QFrame;
  QGridLayout *frameLayout = new QGridLayout;

  // Create the widgets for reading the Oculus window coordinates.
  QLabel *xLabel = new QLabel("X:");
  xEdit = new QLineEdit();
  xEdit->setFixedWidth(45);
  xEdit->setMaxLength(4);
  xEdit->setValidator( new QIntValidator(0, 9999, this));
  QLabel *yLabel = new QLabel("Y:");
  yEdit = new QLineEdit();
  yEdit->setFixedWidth(45);
  yEdit->setMaxLength(4);
  yEdit->setValidator( new QIntValidator(0, 9999, this));

  // Create the widget for reading the visual link where the Oculus will
  // be attached.
  QLabel *visualLabel = new QLabel("Visual:");
  visualEdit = new QLineEdit();

  // Create the widgets for reading the offset applied to the Oculus.
  QLabel *xOffsetLabel = new QLabel("X offset:");
  xOffsetEdit = new QLineEdit();
  xOffsetEdit->setFixedWidth(45);
  xOffsetEdit->setMaxLength(10);
  xOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *yOffsetLabel = new QLabel("Y offset:");
  yOffsetEdit = new QLineEdit();
  yOffsetEdit->setFixedWidth(45);
  yOffsetEdit->setMaxLength(10);
  yOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *zOffsetLabel = new QLabel("Z offset:");
  zOffsetEdit = new QLineEdit();
  zOffsetEdit->setFixedWidth(45);
  zOffsetEdit->setMaxLength(10);
  zOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *rollOffsetLabel = new QLabel("Roll offset:");
  rollOffsetEdit = new QLineEdit();
  rollOffsetEdit->setFixedWidth(45);
  rollOffsetEdit->setMaxLength(10);
  rollOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *pitchOffsetLabel = new QLabel("Pitch offset:");
  pitchOffsetEdit = new QLineEdit();
  pitchOffsetEdit->setFixedWidth(45);
  pitchOffsetEdit->setMaxLength(10);
  pitchOffsetEdit->setValidator( new QDoubleValidator());
  QLabel *yawOffsetLabel = new QLabel("Yaw offset:");
  yawOffsetEdit = new QLineEdit();
  yawOffsetEdit->setFixedWidth(45);
  yawOffsetEdit->setMaxLength(10);
  yawOffsetEdit->setValidator( new QDoubleValidator());

  // Inset all the widgets into the grid layout.
  frameLayout->addWidget(xLabel, 0, 1);
  frameLayout->addWidget(xEdit, 0, 2);
  frameLayout->addWidget(yLabel, 1, 1);
  frameLayout->addWidget(yEdit, 1, 2);
  frameLayout->addWidget(visualLabel, 2, 1);
  frameLayout->addWidget(visualEdit, 2, 2);
  frameLayout->addWidget(xOffsetLabel, 3, 1);
  frameLayout->addWidget(xOffsetEdit, 3, 2);
  frameLayout->addWidget(yOffsetLabel, 3, 3);
  frameLayout->addWidget(yOffsetEdit, 3, 4);
  frameLayout->addWidget(zOffsetLabel, 3, 5);
  frameLayout->addWidget(zOffsetEdit, 3, 6);
  frameLayout->addWidget(rollOffsetLabel, 4, 1);
  frameLayout->addWidget(rollOffsetEdit, 4, 2);
  frameLayout->addWidget(pitchOffsetLabel, 4, 3);
  frameLayout->addWidget(pitchOffsetEdit, 4, 4);
  frameLayout->addWidget(yawOffsetLabel, 4, 5);
  frameLayout->addWidget(yawOffsetEdit, 4, 6);
  frameLayout->setContentsMargins(4, 4, 4, 4);
  frame->setLayout(frameLayout);

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton("Cancel");
  connect(cancelButton, SIGNAL(clicked()),
          this, SLOT(OnCancel()));

  this->okayButton = new QPushButton("Okay");
  //this->okayButton->setEnabled(false);
  connect(this->okayButton, SIGNAL(clicked()),
          this, SLOT(OnOkay()));

  buttonLayout->addWidget(cancelButton);
  buttonLayout->addStretch(2);
  buttonLayout->addWidget(this->okayButton);

  mainLayout->addWidget(frame);
  mainLayout->addLayout(buttonLayout);

  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
OculusConfig::~OculusConfig()
{

}

/////////////////////////////////////////////////
void OculusConfig::Update()
{
  this->oculusWindowCoordinates.Set(
    this->xEdit->text().toDouble(),
    this->yEdit->text().toDouble());

  this->visual = this->visualEdit->text().toUtf8().constData();

  math::Vector3 offsetPos = math::Vector3(
    this->xOffsetEdit->text().toDouble(),
    this->yOffsetEdit->text().toDouble(),
    this->zOffsetEdit->text().toDouble());

  math::Quaternion offsetRot = math::Quaternion(
    this->rollOffsetEdit->text().toDouble(),
    this->pitchOffsetEdit->text().toDouble(),
    this->yawOffsetEdit->text().toDouble());

  this->offset.Set(offsetPos, offsetRot);
}

/////////////////////////////////////////////////
void OculusConfig::OnOkay()
{
  this->done(QDialog::Accepted);
  this->Update();
}

/////////////////////////////////////////////////
void OculusConfig::OnCancel()
{
  this->done(QDialog::Rejected);
}

/////////////////////////////////////////////////
math::Vector2d OculusConfig::GetOculuswindowCoordinates()
{
  return this->oculusWindowCoordinates;
}

/////////////////////////////////////////////////
std::string OculusConfig::GetVisual()
{
  return this->visual;
}

/////////////////////////////////////////////////
math::Pose OculusConfig::GetOffset()
{
  return this->offset;
}
