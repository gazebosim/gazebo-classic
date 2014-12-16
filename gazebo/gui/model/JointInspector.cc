/*
 * Copyright (C) 2013-2014 Open Source Robotics Foundation
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
#include "gazebo/common/Assert.hh"

#include "gazebo/gui/model/JointInspector.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointInspector::JointInspector(JointMaker::JointType _jointType,
  QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("JointInspectorDialog");
  this->setWindowTitle(tr("Joint Inspector"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  this->jointType = _jointType;

  QLabel *jointLabel = new QLabel(tr("Name:"));
  this->jointNameLineEdit = new QLineEdit(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(jointLabel);
  nameLayout->addWidget(this->jointNameLineEdit);

  QLabel *typeLabel = new QLabel(tr("Type: "));
  this->jointTypeComboBox = new QComboBox;
  this->jointTypeComboBox->addItem("revolute",
    QVariant(JointMaker::JOINT_HINGE));
  this->jointTypeComboBox->addItem("revolute2",
      QVariant(JointMaker::JOINT_HINGE2));
  this->jointTypeComboBox->addItem("prismatic",
      QVariant(JointMaker::JOINT_SLIDER));
  this->jointTypeComboBox->addItem("ball",
      QVariant(JointMaker::JOINT_BALL));
  this->jointTypeComboBox->addItem("universal",
      QVariant(JointMaker::JOINT_UNIVERSAL));
  this->jointTypeComboBox->addItem("screw",
      QVariant(JointMaker::JOINT_SCREW));
  connect(this->jointTypeComboBox, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnJointTypeChanged(int)));

  QHBoxLayout *typeLayout = new QHBoxLayout;
  typeLayout->addWidget(typeLabel);
  typeLayout->addWidget(this->jointTypeComboBox);

  QLabel *parentLabel = new QLabel(tr("Parent: "));
  this->jointParentLabel = new QLabel(tr(""));

  QHBoxLayout *parentLayout = new QHBoxLayout;
  parentLayout->addWidget(parentLabel);
  parentLayout->addWidget(jointParentLabel);

  QLabel *childLabel = new QLabel(tr("Child: "));
  this->jointChildLabel = new QLabel(tr(""));

  QHBoxLayout *childLayout = new QHBoxLayout;
  childLayout->addWidget(childLabel);
  childLayout->addWidget(jointChildLabel);

  QLabel *poseXLabel = new QLabel(tr("x: "));
  QLabel *poseYLabel = new QLabel(tr("y: "));
  QLabel *poseZLabel = new QLabel(tr("z: "));
  QLabel *poseRollLabel = new QLabel(tr("roll: "));
  QLabel *posePitchLabel = new QLabel(tr("pitch: "));
  QLabel *poseYawLabel = new QLabel(tr("yaw: "));

  this->poseXSpinBox = new QDoubleSpinBox;
  this->poseXSpinBox->setRange(-1000, 1000);
  this->poseXSpinBox->setSingleStep(0.01);
  this->poseXSpinBox->setDecimals(3);
  this->poseXSpinBox->setValue(0.000);

  this->poseYSpinBox = new QDoubleSpinBox;
  this->poseYSpinBox->setRange(-1000, 1000);
  this->poseYSpinBox->setSingleStep(0.01);
  this->poseYSpinBox->setDecimals(3);
  this->poseYSpinBox->setValue(0.000);

  this->poseZSpinBox = new QDoubleSpinBox;
  this->poseZSpinBox->setRange(-1000, 1000);
  this->poseZSpinBox->setSingleStep(0.01);
  this->poseZSpinBox->setDecimals(3);
  this->poseZSpinBox->setValue(0.000);

  this->poseRollSpinBox = new QDoubleSpinBox;
  this->poseRollSpinBox->setRange(-1000, 1000);
  this->poseRollSpinBox->setSingleStep(0.01);
  this->poseRollSpinBox->setDecimals(3);
  this->poseRollSpinBox->setValue(0.000);

  this->posePitchSpinBox = new QDoubleSpinBox;
  this->posePitchSpinBox->setRange(-1000, 1000);
  this->posePitchSpinBox->setSingleStep(0.01);
  this->posePitchSpinBox->setDecimals(3);
  this->posePitchSpinBox->setValue(0.000);

  this->poseYawSpinBox = new QDoubleSpinBox;
  this->poseYawSpinBox->setRange(-1000, 1000);
  this->poseYawSpinBox->setSingleStep(0.01);
  this->poseYawSpinBox->setDecimals(3);
  this->poseYawSpinBox->setValue(0.000);

  QGridLayout *poseLayout = new QGridLayout;
  poseLayout->addWidget(poseXLabel, 0, 0);
  poseLayout->addWidget(poseXSpinBox, 0, 1);
  poseLayout->addWidget(poseYLabel, 1, 0);
  poseLayout->addWidget(poseYSpinBox, 1, 1);
  poseLayout->addWidget(poseZLabel, 2, 0);
  poseLayout->addWidget(poseZSpinBox, 2, 1);
  poseLayout->addWidget(poseRollLabel, 0, 2);
  poseLayout->addWidget(poseRollSpinBox, 0, 3);
  poseLayout->addWidget(posePitchLabel, 1, 2);
  poseLayout->addWidget(posePitchSpinBox, 1, 3);
  poseLayout->addWidget(poseYawLabel, 2, 2);
  poseLayout->addWidget(poseYawSpinBox, 2, 3);

  QGroupBox *poseGroupBox = new QGroupBox(tr("Pose"));
  poseGroupBox->setLayout(poseLayout);

  int axisCount = 2;
  for (int i = 0; i < axisCount; ++i)
  {
    QLabel *axisXLabel = new QLabel(tr("x: "));
    QLabel *axisYLabel = new QLabel(tr("y: "));
    QLabel *axisZLabel = new QLabel(tr("z: "));

    QDoubleSpinBox *axisXSpinBox = new QDoubleSpinBox;
    axisXSpinBox->setRange(-1000, 1000);
    axisXSpinBox->setSingleStep(0.01);
    axisXSpinBox->setDecimals(3);
    axisXSpinBox->setValue(0.000);
    this->axisXSpinBoxes.push_back(axisXSpinBox);

    QDoubleSpinBox *axisYSpinBox = new QDoubleSpinBox;
    axisYSpinBox->setRange(-1000, 1000);
    axisYSpinBox->setSingleStep(0.01);
    axisYSpinBox->setDecimals(3);
    axisYSpinBox->setValue(0.000);
    this->axisYSpinBoxes.push_back(axisYSpinBox);

    QDoubleSpinBox *axisZSpinBox = new QDoubleSpinBox;
    axisZSpinBox->setRange(-1000, 1000);
    axisZSpinBox->setSingleStep(0.01);
    axisZSpinBox->setDecimals(3);
    axisZSpinBox->setValue(0.000);
    this->axisZSpinBoxes.push_back(axisZSpinBox);

    QGridLayout *axisLayout = new QGridLayout;
    axisLayout->addWidget(axisXLabel, 0, 0);
    axisLayout->addWidget(axisXSpinBox, 0, 1);
    axisLayout->addWidget(axisYLabel), 1, 0;
    axisLayout->addWidget(axisYSpinBox, 1, 1);
    axisLayout->addWidget(axisZLabel), 2, 0;
    axisLayout->addWidget(axisZSpinBox, 2, 1);

    QLabel *lowerLimitLabel = new QLabel(tr("Lower: "));
    QLabel *upperLimitLabel = new QLabel(tr("Upper: "));

    QDoubleSpinBox *lowerLimitSpinBox = new QDoubleSpinBox;
    lowerLimitSpinBox->setRange(-1000, 1000);
    lowerLimitSpinBox->setSingleStep(0.01);
    lowerLimitSpinBox->setDecimals(3);
    this->lowerLimitSpinBoxes.push_back(lowerLimitSpinBox);

    QDoubleSpinBox *upperLimitSpinBox = new QDoubleSpinBox;
    upperLimitSpinBox->setRange(-1000, 1000);
    upperLimitSpinBox->setSingleStep(0.01);
    upperLimitSpinBox->setDecimals(3);
    this->upperLimitSpinBoxes.push_back(upperLimitSpinBox);

    QGridLayout *limitLayout = new QGridLayout;
    limitLayout->addWidget(lowerLimitLabel, 0, 0);
    limitLayout->addWidget(lowerLimitSpinBox, 0, 1);
    limitLayout->addWidget(upperLimitLabel), 1, 0;
    limitLayout->addWidget(upperLimitSpinBox, 1, 1);

    QGroupBox *limitGroupBox = new QGroupBox(tr("Limit"));
    limitGroupBox->setLayout(limitLayout);

    QVBoxLayout *axisAllLayout = new QVBoxLayout;
    axisAllLayout->addLayout(axisLayout);
    axisAllLayout->addWidget(limitGroupBox);

    std::stringstream ss;
    ss << "Axis" << (i+1);
    QGroupBox *axisGroupBox = new QGroupBox(tr(ss.str().c_str()));
    axisGroupBox->setLayout(axisAllLayout);
    this->axisGroupBoxes.push_back(axisGroupBox);
  }

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
  mainLayout->addLayout(typeLayout);
  mainLayout->addLayout(parentLayout);
  mainLayout->addLayout(childLayout);
  mainLayout->addWidget(poseGroupBox);
  for (unsigned int i = 0; i < axisGroupBoxes.size(); ++i)
  {
    mainLayout->addWidget(this->axisGroupBoxes[i]);
    this->axisGroupBoxes[i]->setVisible(false);
  }
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);

  if (this->jointType)
    this->SetType(this->jointType);
}

/////////////////////////////////////////////////
JointInspector::~JointInspector()
{
}

/////////////////////////////////////////////////
math::Pose JointInspector::GetPose() const
{
  return math::Pose(this->poseXSpinBox->value(),
      this->poseYSpinBox->value(), this->poseZSpinBox->value(),
      this->poseRollSpinBox->value(), this->posePitchSpinBox->value(),
      this->poseYawSpinBox->value());
}

/////////////////////////////////////////////////
math::Vector3 JointInspector::GetAxis(unsigned int _index) const
{
  if (_index > this->axisXSpinBoxes.size())
  {
    gzerr << "Axis index is out of range" << std::endl;
    return math::Vector3::Zero;
  }

  return math::Vector3(this->axisXSpinBoxes[_index]->value(),
      this->axisYSpinBoxes[_index]->value(),
      this->axisZSpinBoxes[_index]->value());
}

/////////////////////////////////////////////////
double JointInspector::GetLowerLimit(unsigned int _index) const
{
  if (_index > this->lowerLimitSpinBoxes.size())
  {
    gzerr << "Axis index is out of range" << std::endl;
    return 0;
  }

  return this->lowerLimitSpinBoxes[_index]->value();
}

/////////////////////////////////////////////////
double JointInspector::GetUpperLimit(unsigned int _index) const
{
  if (_index > this->upperLimitSpinBoxes.size())
  {
    gzerr << "Axis index is out of range" << std::endl;
    return 0;
  }

  return this->upperLimitSpinBoxes[_index]->value();
}

/////////////////////////////////////////////////
JointMaker::JointType JointInspector::GetType() const
{
  return this->jointType;
}
/////////////////////////////////////////////////
std::string JointInspector::GetName() const
{
  return this->jointNameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
void JointInspector::SetType(JointMaker::JointType _type)
{
  this->jointType =  _type;

  std::string jointTypeStr = JointMaker::GetTypeAsString(_type);
  int axisCount = JointMaker::GetJointAxisCount(_type);
  GZ_ASSERT(axisCount >= 0, "Invalid axis count");


  int index = this->jointTypeComboBox->findText(tr(jointTypeStr.c_str()));

  if (index >= 0)
    this->jointTypeComboBox->setCurrentIndex(index);
  else
  {
    gzerr << "Joint type not found in inspector" << std::endl;
    return;
  }

  for (int i = 0; i < axisCount; ++i)
    this->axisGroupBoxes[i]->setVisible(true);

  for (int i = axisCount;
      i < static_cast<int>(this->axisGroupBoxes.size()); ++i)
  {
    this->axisGroupBoxes[i]->setVisible(false);
  }
}

/////////////////////////////////////////////////
void JointInspector::SetName(const std::string &_name)
{
  this->jointNameLineEdit->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void JointInspector::SetParent(const std::string &_parent)
{
  this->jointParentLabel->setText(tr(_parent.c_str()));
}

/////////////////////////////////////////////////
void JointInspector::SetChild(const std::string &_child)
{
  this->jointChildLabel->setText(tr(_child.c_str()));
}

/////////////////////////////////////////////////
void JointInspector::SetPose(const math::Pose &_pose)
{
  this->poseXSpinBox->setValue(_pose.pos.x);
  this->poseYSpinBox->setValue(_pose.pos.y);
  this->poseZSpinBox->setValue(_pose.pos.z);

  math::Vector3 rot = _pose.rot.GetAsEuler();
  this->poseRollSpinBox->setValue(rot.x);
  this->posePitchSpinBox->setValue(rot.y);
  this->poseYawSpinBox->setValue(rot.z);
}

/////////////////////////////////////////////////
void JointInspector::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  if (_index > this->axisXSpinBoxes.size())
  {
    gzerr << "Axis index is out of range" << std::endl;
    return;
  }

  this->axisXSpinBoxes[_index]->setValue(_axis.x);
  this->axisYSpinBoxes[_index]->setValue(_axis.y);
  this->axisZSpinBoxes[_index]->setValue(_axis.z);
}

/////////////////////////////////////////////////
void JointInspector::SetLowerLimit(unsigned int _index, double _lower)
{
  if (_index > this->lowerLimitSpinBoxes.size())
  {
    gzerr << "Axis index is out of range" << std::endl;
    return;
  }

  this->lowerLimitSpinBoxes[_index]->setValue(_lower);
}

/////////////////////////////////////////////////
void JointInspector::SetUpperLimit(unsigned int _index, double _upper)
{
  if (_index > this->upperLimitSpinBoxes.size())
  {
    gzerr << "Axis index is out of range" << std::endl;
    return;
  }

  this->upperLimitSpinBoxes[_index]->setValue(_upper);
}

/////////////////////////////////////////////////
void JointInspector::OnJointTypeChanged(int _index)
{
  QVariant jointTypeData = this->jointTypeComboBox->itemData(_index);
  this->jointType = static_cast<JointMaker::JointType>(jointTypeData.toInt());

  int axisCount = JointMaker::GetJointAxisCount(this->jointType);
  for (int i = 0; i < axisCount; ++i)
    this->axisGroupBoxes[i]->setVisible(true);

  for (int i = axisCount;
      i < static_cast<int>(this->axisGroupBoxes.size()); ++i)
  {
    this->axisGroupBoxes[i]->setVisible(false);
  }
}

/////////////////////////////////////////////////
void JointInspector::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void JointInspector::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnOK()
{
  emit Applied();
  this->accept();
}
