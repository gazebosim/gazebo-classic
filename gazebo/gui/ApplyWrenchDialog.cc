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

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ApplyWrenchDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ApplyWrenchDialog::ApplyWrenchDialog(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("ApplyWrenchDialog");

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
  this->forceMagSpin->setRange(0, GZ_DBL_MAX);
  this->forceMagSpin->setSingleStep(0.1);
  this->forceMagSpin->setDecimals(3);
  this->forceMagSpin->setValue(1000);
  connect(this->forceMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceMagChanged(double)));

  // Force direction
  QLabel *forceDirectionLabel = new QLabel();
  forceDirectionLabel->setText(tr("Direction:"));

  // Force X
  QLabel *forceXLabel = new QLabel();
  forceXLabel->setText(tr("X:"));
  QLabel *forceXUnitLabel = new QLabel();
  forceXUnitLabel->setText(tr("N"));

  this->forceXSpin = new QDoubleSpinBox();
  this->forceXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->forceXSpin->setSingleStep(0.1);
  this->forceXSpin->setDecimals(3);
  this->forceXSpin->setValue(1);
  connect(this->forceXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceXChanged(double)));

  // Force Y
  QLabel *forceYLabel = new QLabel();
  forceYLabel->setText(tr("Y:"));
  QLabel *forceYUnitLabel = new QLabel();
  forceYUnitLabel->setText(tr("N"));

  this->forceYSpin = new QDoubleSpinBox();
  this->forceYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->forceYSpin->setSingleStep(0.1);
  this->forceYSpin->setDecimals(3);
  this->forceYSpin->setValue(0);
  connect(this->forceYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceYChanged(double)));

  // Force Z
  QLabel *forceZLabel = new QLabel();
  forceZLabel->setText(tr("Z:"));
  QLabel *forceZUnitLabel = new QLabel();
  forceZUnitLabel->setText(tr("N"));

  this->forceZSpin = new QDoubleSpinBox();
  this->forceZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
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
  forceLayout->addWidget(forceXUnitLabel, 3, 2);
  forceLayout->addWidget(forceYLabel, 4, 0);
  forceLayout->addWidget(this->forceYSpin, 4, 1);
  forceLayout->addWidget(forceYUnitLabel, 4, 2);
  forceLayout->addWidget(forceZLabel, 5, 0);
  forceLayout->addWidget(this->forceZSpin, 5, 1);
  forceLayout->addWidget(forceZUnitLabel, 5, 2);

  // Torque
  QLabel *torqueLabel = new QLabel();
  torqueLabel->setText(tr("Torque"));

  // Torque magnitude
  QLabel *torqueMagLabel = new QLabel();
  torqueMagLabel->setText(tr("Magnitude:"));
  QLabel *torqueMagUnitLabel = new QLabel();
  torqueMagUnitLabel->setText(tr("Nm"));

  this->torqueMagSpin = new QDoubleSpinBox();
  this->torqueMagSpin->setRange(0, GZ_DBL_MAX);
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
  QLabel *torqueXUnitLabel = new QLabel();
  torqueXUnitLabel->setText(tr("Nm"));

  this->torqueXSpin = new QDoubleSpinBox();
  this->torqueXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->torqueXSpin->setSingleStep(0.1);
  this->torqueXSpin->setDecimals(3);
  this->torqueXSpin->setValue(0);
  connect(this->torqueXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueXChanged(double)));

  // Torque Y
  QLabel *torqueYLabel = new QLabel();
  torqueYLabel->setText(tr("Y:"));
  QLabel *torqueYUnitLabel = new QLabel();
  torqueYUnitLabel->setText(tr("Nm"));

  this->torqueYSpin = new QDoubleSpinBox();
  this->torqueYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->torqueYSpin->setSingleStep(0.1);
  this->torqueYSpin->setDecimals(3);
  this->torqueYSpin->setValue(0);
  connect(this->torqueYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueYChanged(double)));

  // Torque Z
  QLabel *torqueZLabel = new QLabel();
  torqueZLabel->setText(tr("Z:"));
  QLabel *torqueZUnitLabel = new QLabel();
  torqueZUnitLabel->setText(tr("Nm"));

  this->torqueZSpin = new QDoubleSpinBox();
  this->torqueZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
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
  torqueLayout->addWidget(torqueXUnitLabel, 3, 2);
  torqueLayout->addWidget(torqueYLabel, 4, 0);
  torqueLayout->addWidget(this->torqueYSpin, 4, 1);
  torqueLayout->addWidget(torqueYUnitLabel, 4, 2);
  torqueLayout->addWidget(torqueZLabel, 5, 0);
  torqueLayout->addWidget(this->torqueZSpin, 5, 1);
  torqueLayout->addWidget(torqueZUnitLabel, 5, 2);

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

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->UpdateForceVector();
  this->UpdateTorqueVector();
  this->CalculateWrench();
}

/////////////////////////////////////////////////
ApplyWrenchDialog::~ApplyWrenchDialog()
{
  this->node->Fini();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetModel(std::string _modelName)
{
  this->messageLabel->setText(
      tr("Apply Force and Torque")); // add model name
  this->modelName = _modelName;
  this->SetPublisher();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnApply()
{
  // publish wrench msg
  msgs::Wrench msg;
  msgs::Set(msg.mutable_force(), this->forceVector);
  msgs::Set(msg.mutable_torque(), this->torqueVector);

  this->wrenchPub->Publish(msg);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnCancel()
{
  this->reject();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceMagChanged(double /*_magnitude*/)
{
  this->UpdateForceVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceXChanged(double /*_fX*/)
{
  this->UpdateForceMag();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceYChanged(double /*_fY*/)
{
  this->UpdateForceMag();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceZChanged(double /*_fZ*/)
{
  this->UpdateForceMag();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueMagChanged(double /*_magnitude*/)
{
  this->UpdateTorqueVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueXChanged(double /*_fX*/)
{
  this->UpdateTorqueMag();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueYChanged(double /*_fY*/)
{
  this->UpdateTorqueMag();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueZChanged(double /*_fZ*/)
{
  this->UpdateTorqueMag();
}

//////////////////////////////////////////////////
void ApplyWrenchDialog::SetPublisher()
{
  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
      GetVisual(this->modelName);

  std::string linkName = this->modelName;
  // If the visual is a model, get its canonical link
  // For now getting the first link that comes up
  if (vis && vis == vis->GetRootVisual())
  {
    linkName = vis->GetChild(0)->GetName();
  }

  this->topicName = "~/";
  topicName += linkName + "/wrench";
  boost::replace_all(topicName, "::", "/");

  this->wrenchPub = this->node->Advertise<msgs::Wrench>(this->topicName);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::CalculateWrench()
{
  this->forceVector =
      math::Vector3(this->forceXSpin->value(),
                    this->forceYSpin->value(),
                    this->forceZSpin->value());
  this->torqueVector =
      math::Vector3(this->torqueXSpin->value(),
                    this->torqueYSpin->value(),
                    this->torqueZSpin->value());
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceMag()
{
  this->forceMagSpin->setValue(sqrt(
      pow(this->forceXSpin->value(), 2) +
      pow(this->forceYSpin->value(), 2) +
      pow(this->forceZSpin->value(), 2)));
  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceVector()
{
  // Normalize current vector
  math::Vector3 v = math::Vector3(this->forceXSpin->value(),
                                   this->forceYSpin->value(),
                                   this->forceZSpin->value());
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  v = v * this->forceMagSpin->value();

  // Update spins
  this->forceXSpin->setValue(v.x);
  this->forceYSpin->setValue(v.y);
  this->forceZSpin->setValue(v.z);

  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueMag()
{
  this->torqueMagSpin->setValue(sqrt(
      pow(this->torqueXSpin->value(), 2) +
      pow(this->torqueYSpin->value(), 2) +
      pow(this->torqueZSpin->value(), 2)));
  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueVector()
{
  // Normalize current vector
  math::Vector3 v = math::Vector3(this->torqueXSpin->value(),
                                   this->torqueYSpin->value(),
                                   this->torqueZSpin->value());
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  v = v * this->torqueMagSpin->value();

  // Update spins
  this->torqueXSpin->setValue(v.x);
  this->torqueYSpin->setValue(v.y);
  this->torqueZSpin->setValue(v.z);

  this->CalculateWrench();
}
