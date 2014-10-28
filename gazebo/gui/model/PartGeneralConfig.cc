/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/gui/model/PartGeneralConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PartGeneralConfig::PartGeneralConfig()
{
  this->setObjectName("PartGeneralConfig");
  QVBoxLayout *generalLayout = new QVBoxLayout;

  QLabel *gravityLabel = new QLabel(tr("Gravity:"));
  this->gravityCheck = new QCheckBox;
  this->gravityCheck->setText(tr("True"));
  this->gravityCheck->setChecked(true);
  connect(this->gravityCheck, SIGNAL(clicked()), this, SLOT(OnGravity()));

  QLabel *selfCollideLabel = new QLabel(tr("Self Collide:"));
  this->selfCollideCheck = new QCheckBox;
  this->selfCollideCheck->setText(tr("False"));
  this->selfCollideCheck->setChecked(false);
  connect(this->selfCollideCheck, SIGNAL(clicked()), this,
      SLOT(OnSelfCollide()));

  QLabel *kinematicLabel = new QLabel(tr("Kinematic:"));
  this->kinematicCheck = new QCheckBox;
  this->kinematicCheck->setText(tr("False"));
  this->kinematicCheck->setChecked(false);
  connect(this->kinematicCheck, SIGNAL(clicked()), this,
      SLOT(OnKinematic()));

  QGridLayout *partLayout = new QGridLayout;
  partLayout->addWidget(gravityLabel, 0, 0);
  partLayout->addWidget(this->gravityCheck, 0, 1);
  partLayout->addWidget(selfCollideLabel, 1, 0);
  partLayout->addWidget(this->selfCollideCheck, 1, 1);
  partLayout->addWidget(kinematicLabel, 2, 0);
  partLayout->addWidget(this->kinematicCheck, 2, 1);

  QLabel *posXLabel = new QLabel(tr("x: "));
  QLabel *posYLabel = new QLabel(tr("y: "));
  QLabel *posZLabel = new QLabel(tr("z: "));
  QLabel *rotRLabel = new QLabel(tr("roll: "));
  QLabel *rotPLabel = new QLabel(tr("pitch: "));
  QLabel *rotYLabel = new QLabel(tr("yaw: "));

  this->posXSpinBox = new QDoubleSpinBox;
  this->posXSpinBox->setRange(-1000, 1000);
  this->posXSpinBox->setSingleStep(0.01);
  this->posXSpinBox->setDecimals(3);
  this->posXSpinBox->setValue(0.000);

  this->posYSpinBox = new QDoubleSpinBox;
  this->posYSpinBox->setRange(-1000, 1000);
  this->posYSpinBox->setSingleStep(0.01);
  this->posYSpinBox->setDecimals(3);
  this->posYSpinBox->setValue(0.000);

  this->posZSpinBox = new QDoubleSpinBox;
  this->posZSpinBox->setRange(-1000, 1000);
  this->posZSpinBox->setSingleStep(0.01);
  this->posZSpinBox->setDecimals(3);
  this->posZSpinBox->setValue(0.000);

  this->rotRSpinBox = new QDoubleSpinBox;
  this->rotRSpinBox->setRange(-1000, 1000);
  this->rotRSpinBox->setSingleStep(0.01);
  this->rotRSpinBox->setDecimals(3);
  this->rotRSpinBox->setValue(0.000);

  this->rotPSpinBox = new QDoubleSpinBox;
  this->rotPSpinBox->setRange(-1000, 1000);
  this->rotPSpinBox->setSingleStep(0.01);
  this->rotPSpinBox->setDecimals(3);
  this->rotPSpinBox->setValue(0.000);

  this->rotYSpinBox = new QDoubleSpinBox;
  this->rotYSpinBox->setRange(-1000, 1000);
  this->rotYSpinBox->setSingleStep(0.01);
  this->rotYSpinBox->setDecimals(3);
  this->rotYSpinBox->setValue(0.000);

  QGridLayout *poseGroupLayout = new QGridLayout;
  poseGroupLayout->addWidget(posXLabel, 0, 0);
  poseGroupLayout->addWidget(posXSpinBox, 0, 1);
  poseGroupLayout->addWidget(posYLabel, 0, 2);
  poseGroupLayout->addWidget(posYSpinBox, 0, 3);
  poseGroupLayout->addWidget(posZLabel, 0, 4);
  poseGroupLayout->addWidget(posZSpinBox, 0, 5);
  poseGroupLayout->addWidget(rotRLabel, 1, 0);
  poseGroupLayout->addWidget(rotRSpinBox, 1, 1);
  poseGroupLayout->addWidget(rotPLabel, 1, 2);
  poseGroupLayout->addWidget(rotPSpinBox, 1, 3);
  poseGroupLayout->addWidget(rotYLabel, 1, 4);
  poseGroupLayout->addWidget(rotYSpinBox, 1, 5);

  poseGroupLayout->setColumnStretch(1, 1);
  poseGroupLayout->setAlignment(posXSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(posYSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(posZSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(rotRSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(rotPSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(rotYSpinBox, Qt::AlignLeft);

  QGroupBox *poseGroupBox = new QGroupBox(tr("Pose"));
  poseGroupBox->setLayout(poseGroupLayout);

  QLabel *inertialPosXLabel = new QLabel(tr("x: "));
  QLabel *inertialPosYLabel = new QLabel(tr("y: "));
  QLabel *inertialPosZLabel = new QLabel(tr("z: "));
  QLabel *inertialRotRLabel = new QLabel(tr("roll: "));
  QLabel *inertialRotPLabel = new QLabel(tr("pitch: "));
  QLabel *inertialRotYLabel = new QLabel(tr("yaw: "));

  this->inertialPosXSpinBox = new QDoubleSpinBox;
  this->inertialPosXSpinBox->setRange(-1000, 1000);
  this->inertialPosXSpinBox->setSingleStep(0.01);
  this->inertialPosXSpinBox->setDecimals(3);
  this->inertialPosXSpinBox->setValue(0.000);

  this->inertialPosYSpinBox = new QDoubleSpinBox;
  this->inertialPosYSpinBox->setRange(-1000, 1000);
  this->inertialPosYSpinBox->setSingleStep(0.01);
  this->inertialPosYSpinBox->setDecimals(3);
  this->inertialPosYSpinBox->setValue(0.000);

  this->inertialPosZSpinBox = new QDoubleSpinBox;
  this->inertialPosZSpinBox->setRange(-1000, 1000);
  this->inertialPosZSpinBox->setSingleStep(0.01);
  this->inertialPosZSpinBox->setDecimals(3);
  this->inertialPosZSpinBox->setValue(0.000);

  this->inertialRotRSpinBox = new QDoubleSpinBox;
  this->inertialRotRSpinBox->setRange(-1000, 1000);
  this->inertialRotRSpinBox->setSingleStep(0.01);
  this->inertialRotRSpinBox->setDecimals(3);
  this->inertialRotRSpinBox->setValue(0.000);

  this->inertialRotPSpinBox = new QDoubleSpinBox;
  this->inertialRotPSpinBox->setRange(-1000, 1000);
  this->inertialRotPSpinBox->setSingleStep(0.01);
  this->inertialRotPSpinBox->setDecimals(3);
  this->inertialRotPSpinBox->setValue(0.000);

  this->inertialRotYSpinBox = new QDoubleSpinBox;
  this->inertialRotYSpinBox->setRange(-1000, 1000);
  this->inertialRotYSpinBox->setSingleStep(0.01);
  this->inertialRotYSpinBox->setDecimals(3);
  this->inertialRotYSpinBox->setValue(0.000);

  QGridLayout *inertialPoseGroupLayout = new QGridLayout;
  inertialPoseGroupLayout->addWidget(inertialPosXLabel, 0, 0);
  inertialPoseGroupLayout->addWidget(inertialPosXSpinBox, 0, 1);
  inertialPoseGroupLayout->addWidget(inertialPosYLabel, 0, 2);
  inertialPoseGroupLayout->addWidget(inertialPosYSpinBox, 0, 3);
  inertialPoseGroupLayout->addWidget(inertialPosZLabel, 0, 4);
  inertialPoseGroupLayout->addWidget(inertialPosZSpinBox, 0, 5);
  inertialPoseGroupLayout->addWidget(inertialRotRLabel, 1, 0);
  inertialPoseGroupLayout->addWidget(inertialRotRSpinBox, 1, 1);
  inertialPoseGroupLayout->addWidget(inertialRotPLabel, 1, 2);
  inertialPoseGroupLayout->addWidget(inertialRotPSpinBox, 1, 3);
  inertialPoseGroupLayout->addWidget(inertialRotYLabel, 1, 4);
  inertialPoseGroupLayout->addWidget(inertialRotYSpinBox, 1, 5);

  QGroupBox *inertialPoseGroupBox = new QGroupBox(tr("Pose"));
  inertialPoseGroupBox->setLayout(inertialPoseGroupLayout);

  QLabel *massLabel = new QLabel(tr("mass: "));
  this->massSpinBox = new QDoubleSpinBox;
  this->massSpinBox->setRange(-1000, 1000);
  this->massSpinBox->setSingleStep(0.01);
  this->massSpinBox->setDecimals(3);
  this->massSpinBox->setValue(0.000);

  QHBoxLayout *massLayout = new QHBoxLayout;
  massLayout->addWidget(massLabel);
  massLayout->addWidget(massSpinBox);

  QLabel *inertiaIXXLabel = new QLabel(tr("ixx: "));
  QLabel *inertiaIXYLabel = new QLabel(tr("ixy: "));
  QLabel *inertiaIXZLabel = new QLabel(tr("ixz: "));
  QLabel *inertiaIYYLabel = new QLabel(tr("iyy: "));
  QLabel *inertiaIYZLabel = new QLabel(tr("iyz: "));
  QLabel *inertiaIZZLabel = new QLabel(tr("izz: "));

  this->inertiaIXXSpinBox = new QDoubleSpinBox;
  this->inertiaIXXSpinBox->setRange(-1000, 1000);
  this->inertiaIXXSpinBox->setSingleStep(0.01);
  this->inertiaIXXSpinBox->setDecimals(3);
  this->inertiaIXXSpinBox->setValue(0.000);

  this->inertiaIXYSpinBox = new QDoubleSpinBox;
  this->inertiaIXYSpinBox->setRange(-1000, 1000);
  this->inertiaIXYSpinBox->setSingleStep(0.01);
  this->inertiaIXYSpinBox->setDecimals(3);
  this->inertiaIXYSpinBox->setValue(0.000);

  this->inertiaIXZSpinBox = new QDoubleSpinBox;
  this->inertiaIXZSpinBox->setRange(-1000, 1000);
  this->inertiaIXZSpinBox->setSingleStep(0.01);
  this->inertiaIXZSpinBox->setDecimals(3);
  this->inertiaIXZSpinBox->setValue(0.000);

  this->inertiaIYYSpinBox = new QDoubleSpinBox;
  this->inertiaIYYSpinBox->setRange(-1000, 1000);
  this->inertiaIYYSpinBox->setSingleStep(0.01);
  this->inertiaIYYSpinBox->setDecimals(3);
  this->inertiaIYYSpinBox->setValue(0.000);

  this->inertiaIYZSpinBox = new QDoubleSpinBox;
  this->inertiaIYZSpinBox->setRange(-1000, 1000);
  this->inertiaIYZSpinBox->setSingleStep(0.01);
  this->inertiaIYZSpinBox->setDecimals(3);
  this->inertiaIYZSpinBox->setValue(0.000);

  this->inertiaIZZSpinBox = new QDoubleSpinBox;
  this->inertiaIZZSpinBox->setRange(-1000, 1000);
  this->inertiaIZZSpinBox->setSingleStep(0.01);
  this->inertiaIZZSpinBox->setDecimals(3);
  this->inertiaIZZSpinBox->setValue(0.000);

  QGridLayout *inertiaGroupLayout = new QGridLayout;
  inertiaGroupLayout->addWidget(inertiaIXXLabel, 0, 0);
  inertiaGroupLayout->addWidget(inertiaIXXSpinBox, 0, 1);
  inertiaGroupLayout->addWidget(inertiaIYYLabel, 0, 2);
  inertiaGroupLayout->addWidget(inertiaIYYSpinBox, 0, 3);
  inertiaGroupLayout->addWidget(inertiaIZZLabel, 0, 4);
  inertiaGroupLayout->addWidget(inertiaIZZSpinBox, 0, 5);
  inertiaGroupLayout->addWidget(inertiaIXYLabel, 1, 0);
  inertiaGroupLayout->addWidget(inertiaIXYSpinBox, 1, 1);
  inertiaGroupLayout->addWidget(inertiaIXZLabel, 1, 2);
  inertiaGroupLayout->addWidget(inertiaIXZSpinBox, 1, 3);
  inertiaGroupLayout->addWidget(inertiaIYZLabel, 1, 4);
  inertiaGroupLayout->addWidget(inertiaIYZSpinBox, 1, 5);

  QGroupBox *inertiaGroupBox = new QGroupBox(tr("Inertia"));
  inertiaGroupBox->setLayout(inertiaGroupLayout);

  QVBoxLayout *inertialGroupLayout = new QVBoxLayout;
  inertialGroupLayout->addLayout(massLayout);
  inertialGroupLayout->addWidget(inertialPoseGroupBox);
  inertialGroupLayout->addWidget(inertiaGroupBox);

  QGroupBox *inertialGroupBox = new QGroupBox(tr("Inertial"));
  inertialGroupBox->setLayout(inertialGroupLayout);

  generalLayout->addLayout(partLayout);
  generalLayout->addWidget(poseGroupBox);
  generalLayout->addWidget(inertialGroupBox);

  this->setLayout(generalLayout);
}

/////////////////////////////////////////////////
PartGeneralConfig::~PartGeneralConfig()
{
}

/////////////////////////////////////////////////
void PartGeneralConfig::SetGravity(bool _enabled)
{
  this->gravityCheck->setChecked(_enabled);
}

/////////////////////////////////////////////////
bool PartGeneralConfig::GetGravity() const
{
  return this->gravityCheck->isChecked();
}

/////////////////////////////////////////////////
void PartGeneralConfig::SetSelfCollide(bool _enabled)
{
  this->selfCollideCheck->setChecked(_enabled);
}

/////////////////////////////////////////////////
bool PartGeneralConfig::GetSelfCollide() const
{
  return this->selfCollideCheck->isChecked();
}

/////////////////////////////////////////////////
void PartGeneralConfig::SetKinematic(bool _enabled)
{
  this->kinematicCheck->setChecked(_enabled);
}

/////////////////////////////////////////////////
bool PartGeneralConfig::GetKinematic() const
{
  return this->kinematicCheck->isChecked();
}

/////////////////////////////////////////////////
void PartGeneralConfig::SetPose(const math::Pose &_pose)
{
  this->posXSpinBox->setValue(_pose.pos.x);
  this->posYSpinBox->setValue(_pose.pos.y);
  this->posZSpinBox->setValue(_pose.pos.z);

  this->rotRSpinBox->setValue(_pose.rot.GetAsEuler().x);
  this->rotPSpinBox->setValue(_pose.rot.GetAsEuler().y);
  this->rotYSpinBox->setValue(_pose.rot.GetAsEuler().z);
}

/////////////////////////////////////////////////
math::Pose PartGeneralConfig::GetPose() const
{
  return math::Pose(this->posXSpinBox->value(), this->posYSpinBox->value(),
      this->posZSpinBox->value(), this->rotRSpinBox->value(),
      this->rotPSpinBox->value(), this->rotYSpinBox->value());
}

/////////////////////////////////////////////////
void PartGeneralConfig::SetMass(double _mass)
{
  this->massSpinBox->setValue(_mass);
}

/////////////////////////////////////////////////
double PartGeneralConfig::GetMass() const
{
  return this->massSpinBox->value();
}

/////////////////////////////////////////////////
void PartGeneralConfig::SetInertialPose(const math::Pose &_inertialPose)
{
  this->inertialPosXSpinBox->setValue(_inertialPose.pos.x);
  this->inertialPosYSpinBox->setValue(_inertialPose.pos.y);
  this->inertialPosZSpinBox->setValue(_inertialPose.pos.z);

  this->inertialRotRSpinBox->setValue(_inertialPose.rot.GetAsEuler().x);
  this->inertialRotPSpinBox->setValue(_inertialPose.rot.GetAsEuler().y);
  this->inertialRotYSpinBox->setValue(_inertialPose.rot.GetAsEuler().z);
}

/////////////////////////////////////////////////
math::Pose PartGeneralConfig::GetInertialPose() const
{
  return math::Pose(this->inertialPosXSpinBox->value(),
      this->inertialPosYSpinBox->value(), this->inertialPosZSpinBox->value(),
      this->inertialRotRSpinBox->value(), this->inertialRotPSpinBox->value(),
      this->inertialRotYSpinBox->value());
}

/////////////////////////////////////////////////
void PartGeneralConfig::SetInertia(double _ixx, double _iyy, double _izz,
    double _ixy, double _ixz, double _iyz)
{
  this->inertiaIXXSpinBox->setValue(_ixx);
  this->inertiaIYYSpinBox->setValue(_iyy);
  this->inertiaIZZSpinBox->setValue(_izz);
  this->inertiaIXYSpinBox->setValue(_ixy);
  this->inertiaIXZSpinBox->setValue(_ixz);
  this->inertiaIYZSpinBox->setValue(_iyz);
}

/////////////////////////////////////////////////
double PartGeneralConfig::GetInertiaIXX() const
{
  return this->inertiaIXXSpinBox->value();
}

/////////////////////////////////////////////////
double PartGeneralConfig::GetInertiaIYY() const
{
  return this->inertiaIYYSpinBox->value();
}

/////////////////////////////////////////////////
double PartGeneralConfig::GetInertiaIZZ() const
{
  return this->inertiaIZZSpinBox->value();
}

/////////////////////////////////////////////////
double PartGeneralConfig::GetInertiaIXY() const
{
  return this->inertiaIXYSpinBox->value();
}

/////////////////////////////////////////////////
double PartGeneralConfig::GetInertiaIXZ() const
{
  return this->inertiaIXZSpinBox->value();
}

/////////////////////////////////////////////////
double PartGeneralConfig::GetInertiaIYZ() const
{
  return this->inertiaIYZSpinBox->value();
}

/////////////////////////////////////////////////
void PartGeneralConfig::OnGravity()
{
  std::string text = this->gravityCheck->isChecked() ? "True" : "False";
  this->gravityCheck->setText(tr(text.c_str()));
}

/////////////////////////////////////////////////
void PartGeneralConfig::OnSelfCollide()
{
  std::string text = this->selfCollideCheck->isChecked() ? "True" : "False";
  this->selfCollideCheck->setText(tr(text.c_str()));
}

/////////////////////////////////////////////////
void PartGeneralConfig::OnKinematic()
{
  std::string text = this->kinematicCheck->isChecked() ? "True" : "False";
  this->kinematicCheck->setText(tr(text.c_str()));
}
