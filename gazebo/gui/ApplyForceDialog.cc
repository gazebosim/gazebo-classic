/*
 * Copyright 2015 Open Source Robotics Foundation
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
#include "gazebo/rendering/Scene.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ApplyForceDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ApplyForceDialog::ApplyForceDialog(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("ApplyForceDialog");

  this->setWindowTitle(tr("Apply Force and Torque"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);
  this->setWindowModality(Qt::NonModal);

  this->messageLabel = new QLabel();
  this->messageLabel->setText(
      tr("Apply Force and Torque"));

  // Force
  QLabel *forceLabel = new QLabel();
  forceLabel->setText(tr("Force"));

  // Force magnitude
  QLabel *forceMagLabel = new QLabel();
  forceMagLabel->setText(tr("Magnitude:"));
  QLabel *forceMagUnitLabel = new QLabel();
  forceMagUnitLabel->setText(tr("N"));

  this->forceMagSpin = new QDoubleSpinBox();
  this->forceMagSpin->setRange(0.001, 1000);
  this->forceMagSpin->setSingleStep(0.1);
  this->forceMagSpin->setDecimals(3);
  this->forceMagSpin->setValue(1);
  connect(this->forceMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceMagChanged(double)));

  // Force direction
  QLabel *forceDirectionLabel = new QLabel();
  forceDirectionLabel->setText(tr("Direction:"));

  // Force X
  QLabel *forceXLabel = new QLabel();
  forceXLabel->setText(tr("X:"));

  this->forceXSpin = new QDoubleSpinBox();
  this->forceXSpin->setRange(-1, 1);
  this->forceXSpin->setSingleStep(0.1);
  this->forceXSpin->setDecimals(3);
  this->forceXSpin->setValue(-1);
  connect(this->forceXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceXChanged(double)));

  // Force Y
  QLabel *forceYLabel = new QLabel();
  forceYLabel->setText(tr("Y:"));

  this->forceYSpin = new QDoubleSpinBox();
  this->forceYSpin->setRange(-1, 1);
  this->forceYSpin->setSingleStep(0.1);
  this->forceYSpin->setDecimals(3);
  this->forceYSpin->setValue(0);
  connect(this->forceYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceYChanged(double)));

  // Force Z
  QLabel *forceZLabel = new QLabel();
  forceZLabel->setText(tr("Z:"));

  this->forceZSpin = new QDoubleSpinBox();
  this->forceZSpin->setRange(-1, 1);
  this->forceZSpin->setSingleStep(0.1);
  this->forceZSpin->setDecimals(3);
  this->forceZSpin->setValue(0);
  connect(this->forceZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceZChanged(double)));

  QGridLayout *forceLayout = new QGridLayout();
  forceLayout->addWidget(forceLabel, 0, 0);
  forceLayout->addWidget(forceMagLabel, 1, 0);
  forceLayout->addWidget(this->forceMagSpin, 1, 1);
  forceLayout->addWidget(forceMagUnitLabel, 1, 2);
  forceLayout->addWidget(forceDirectionLabel, 2, 0);
  forceLayout->addWidget(forceXLabel, 3, 0);
  forceLayout->addWidget(this->forceXSpin, 3, 1);
  forceLayout->addWidget(forceYLabel, 4, 0);
  forceLayout->addWidget(this->forceYSpin, 4, 1);
  forceLayout->addWidget(forceZLabel, 5, 0);
  forceLayout->addWidget(this->forceZSpin, 5, 1);

  // Torque
  QLabel *torqueLabel = new QLabel();
  torqueLabel->setText(tr("Torque"));

  // Torque magnitude
  QLabel *torqueMagLabel = new QLabel();
  torqueMagLabel->setText(tr("Magnitude:"));
  QLabel *torqueMagUnitLabel = new QLabel();
  torqueMagUnitLabel->setText(tr("N m"));

  this->torqueMagSpin = new QDoubleSpinBox();
  this->torqueMagSpin->setRange(0.001, 1000);
  this->torqueMagSpin->setSingleStep(0.1);
  this->torqueMagSpin->setDecimals(3);
  this->torqueMagSpin->setValue(0);
  connect(this->torqueMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueMagChanged(double)));

  // Torque direction
  QLabel *torqueDirectionLabel = new QLabel();
  torqueDirectionLabel->setText(tr("Direction:"));

  // Torque X
  QLabel *torqueXLabel = new QLabel();
  torqueXLabel->setText(tr("X:"));

  this->torqueXSpin = new QDoubleSpinBox();
  this->torqueXSpin->setRange(-1, 1);
  this->torqueXSpin->setSingleStep(0.1);
  this->torqueXSpin->setDecimals(3);
  this->torqueXSpin->setValue(1);
  connect(this->torqueXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueXChanged(double)));

  // Torque Y
  QLabel *torqueYLabel = new QLabel();
  torqueYLabel->setText(tr("Y:"));

  this->torqueYSpin = new QDoubleSpinBox();
  this->torqueYSpin->setRange(-1, 1);
  this->torqueYSpin->setSingleStep(0.1);
  this->torqueYSpin->setDecimals(3);
  this->torqueYSpin->setValue(0);
  connect(this->torqueYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueYChanged(double)));

  // Torque Z
  QLabel *torqueZLabel = new QLabel();
  torqueZLabel->setText(tr("Z:"));

  this->torqueZSpin = new QDoubleSpinBox();
  this->torqueZSpin->setRange(-1, 1);
  this->torqueZSpin->setSingleStep(0.1);
  this->torqueZSpin->setDecimals(3);
  this->torqueZSpin->setValue(0);
  connect(this->torqueZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueZChanged(double)));

  QGridLayout *torqueLayout = new QGridLayout();
  torqueLayout->addWidget(torqueLabel, 0, 0);
  torqueLayout->addWidget(torqueMagLabel, 1, 0);
  torqueLayout->addWidget(this->torqueMagSpin, 1, 1);
  torqueLayout->addWidget(torqueMagUnitLabel, 1, 2);
  torqueLayout->addWidget(torqueDirectionLabel, 2, 0);
  torqueLayout->addWidget(torqueXLabel, 3, 0);
  torqueLayout->addWidget(this->torqueXSpin, 3, 1);
  torqueLayout->addWidget(torqueYLabel, 4, 0);
  torqueLayout->addWidget(this->torqueYSpin, 4, 1);
  torqueLayout->addWidget(torqueZLabel, 5, 0);
  torqueLayout->addWidget(this->torqueZSpin, 5, 1);

  // Buttons
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *applyButton = new QPushButton("&Apply");
  applyButton->setDefault(true);
  connect(applyButton, SIGNAL(clicked()), this, SLOT(OnApply()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(applyButton);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->messageLabel);
  mainLayout->addLayout(forceLayout);
  mainLayout->addLayout(torqueLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
ApplyForceDialog::~ApplyForceDialog()
{
}

/////////////////////////////////////////////////
void ApplyForceDialog::SetModel(std::string _modelName)
{
  this->messageLabel->setText(
      tr("Apply Force and Torque")); // add model name
  this->modelName = _modelName;
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnApply()
{
  // apply force and torque
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnCancel()
{
  this->reject();
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnForceMagChanged(double _magnitude)
{
  // update visual
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnForceXChanged(double _fX)
{
  // update visual
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnForceYChanged(double _fY)
{
  // update visual
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnForceZChanged(double _fZ)
{
  // update visual
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnTorqueMagChanged(double _magnitude)
{
  // update visual
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnTorqueXChanged(double _fX)
{
  // update visual
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnTorqueYChanged(double _fY)
{
  // update visual
}

/////////////////////////////////////////////////
void ApplyForceDialog::OnTorqueZChanged(double _fZ)
{
  // update visual
}
