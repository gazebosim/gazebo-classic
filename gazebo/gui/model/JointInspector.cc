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

#include "gazebo/common/Console.hh"

#include "gazebo/gui/model/JointInspector.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointInspector::JointInspector(JointMaker::JointType _jointType,
  QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("JointInspectorDialog");
  this->setWindowTitle(tr("Joint Inspector"));

  this->jointType = _jointType;

  QLabel *jointLabel = new QLabel(tr("Name:"));
  this->jointNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(jointLabel);
  nameLayout->addWidget(jointNameLabel);

  QLabel *typeLabel = new QLabel(tr("Type: "));
  this->jointTypeLabel = new QLabel("");
  QHBoxLayout *typeLayout = new QHBoxLayout;
  typeLayout->addWidget(typeLabel);
  typeLayout->addWidget(this->jointTypeLabel);

  QLabel *anchorXLabel = new QLabel(tr("x: "));
  QLabel *anchorYLabel = new QLabel(tr("y: "));
  QLabel *anchorZLabel = new QLabel(tr("z: "));

  this->anchorXSpinBox = new QDoubleSpinBox;
  this->anchorXSpinBox->setRange(-1000, 1000);
  this->anchorXSpinBox->setSingleStep(0.01);
  this->anchorXSpinBox->setDecimals(3);
  this->anchorXSpinBox->setValue(0.000);

  this->anchorYSpinBox = new QDoubleSpinBox;
  this->anchorYSpinBox->setRange(-1000, 1000);
  this->anchorYSpinBox->setSingleStep(0.01);
  this->anchorYSpinBox->setDecimals(3);
  this->anchorYSpinBox->setValue(0.000);

  this->anchorZSpinBox = new QDoubleSpinBox;
  this->anchorZSpinBox->setRange(-1000, 1000);
  this->anchorZSpinBox->setSingleStep(0.01);
  this->anchorZSpinBox->setDecimals(3);
  this->anchorZSpinBox->setValue(0.000);

  QGridLayout *anchorLayout = new QGridLayout;
  anchorLayout->addWidget(anchorXLabel, 0, 0);
  anchorLayout->addWidget(anchorXSpinBox, 0, 1);
  anchorLayout->addWidget(anchorYLabel), 1, 0;
  anchorLayout->addWidget(anchorYSpinBox, 1, 1);
  anchorLayout->addWidget(anchorZLabel), 2, 0;
  anchorLayout->addWidget(anchorZSpinBox, 2, 1);

  QGroupBox *anchorGroupBox = new QGroupBox(tr("Anchor"));
  anchorGroupBox->setLayout(anchorLayout);

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
    axisXSpinBoxes.push_back(axisXSpinBox);

    QDoubleSpinBox *axisYSpinBox = new QDoubleSpinBox;
    axisYSpinBox->setRange(-1000, 1000);
    axisYSpinBox->setSingleStep(0.01);
    axisYSpinBox->setDecimals(3);
    axisYSpinBox->setValue(0.000);
    axisYSpinBoxes.push_back(axisYSpinBox);

    QDoubleSpinBox *axisZSpinBox = new QDoubleSpinBox;
    axisZSpinBox->setRange(-1000, 1000);
    axisZSpinBox->setSingleStep(0.01);
    axisZSpinBox->setDecimals(3);
    axisZSpinBox->setValue(0.000);
    axisZSpinBoxes.push_back(axisZSpinBox);

    QGridLayout *axisLayout = new QGridLayout;
    axisLayout->addWidget(axisXLabel, 0, 0);
    axisLayout->addWidget(axisXSpinBox, 0, 1);
    axisLayout->addWidget(axisYLabel), 1, 0;
    axisLayout->addWidget(axisYSpinBox, 1, 1);
    axisLayout->addWidget(axisZLabel), 2, 0;
    axisLayout->addWidget(axisZSpinBox, 2, 1);

    std::stringstream ss;
    ss << "Axis" << (i+1);
    QGroupBox *axisGroupBox = new QGroupBox(tr(ss.str().c_str()));
    axisGroupBox->setLayout(axisLayout);
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
  mainLayout->addWidget(anchorGroupBox);
  for (unsigned int i = 0; i < axisGroupBoxes.size(); ++i)
  {
    mainLayout->addWidget(this->axisGroupBoxes[i]);
    this->axisGroupBoxes[i]->setVisible(false);
  }
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);

  if (this->jointType)
    this->SetType(this->jointType);
}

/////////////////////////////////////////////////
JointInspector::~JointInspector()
{
}

/////////////////////////////////////////////////
math::Vector3 JointInspector::GetAnchor(int /*_index*/) const
{
  return math::Vector3(this->anchorXSpinBox->value(),
      this->anchorYSpinBox->value(), this->anchorZSpinBox->value());
}

/////////////////////////////////////////////////
math::Vector3 JointInspector::GetAxis(int _index) const
{
  if (_index < 0 || _index > static_cast<int>(this->axisXSpinBoxes.size()))
  {
    gzerr << "Axis index is out of range" << std::endl;
    return math::Vector3::Zero;
  }

  return math::Vector3(this->axisXSpinBoxes[_index]->value(),
      this->axisYSpinBoxes[_index]->value(),
      this->axisZSpinBoxes[_index]->value());
}

/////////////////////////////////////////////////
JointMaker::JointType JointInspector::GetType() const
{
  return this->jointType;
}

/////////////////////////////////////////////////
void JointInspector::SetType(JointMaker::JointType _type)
{
  this->jointType =  _type;

  std::string jointTypeStr = "";
  int axisCount = 0;
  if (this->jointType == JointMaker::JOINT_FIXED)
  {
    jointTypeStr = "Fixed";
    axisCount = 0;
  }
  else if (this->jointType == JointMaker::JOINT_SLIDER)
  {
    jointTypeStr = "Prismatic";
    axisCount = 1;
  }
  else if (this->jointType == JointMaker::JOINT_HINGE)
  {
    jointTypeStr = "Revolute";
    axisCount = 1;
  }
  else if (this->jointType == JointMaker::JOINT_HINGE2)
  {
    jointTypeStr = "Revolute2";
    axisCount = 2;
  }
  else if (this->jointType == JointMaker::JOINT_SCREW)
  {
    jointTypeStr = "Screw";
    axisCount = 1;
  }
  else if (this->jointType == JointMaker::JOINT_UNIVERSAL)
  {
    jointTypeStr = "Universal";
    axisCount = 2;
  }
  else if (this->jointType == JointMaker::JOINT_BALL)
  {
    jointTypeStr = "Ball";
    axisCount = 0;
  }
  this->jointTypeLabel->setText(tr(jointTypeStr.c_str()));

  for (int i = 0; i < axisCount; ++i)
    this->axisGroupBoxes[i]->setVisible(true);

  for (int i = axisCount; i < 2; ++i)
    this->axisGroupBoxes[i]->setVisible(false);
}

/////////////////////////////////////////////////
void JointInspector::SetName(const std::string &_name)
{
  this->jointNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void JointInspector::SetAnchor(int /*_index*/, const math::Vector3 &_anchor)
{
  this->anchorXSpinBox->setValue(_anchor.x);
  this->anchorYSpinBox->setValue(_anchor.y);
  this->anchorZSpinBox->setValue(_anchor.z);
}

/////////////////////////////////////////////////
void JointInspector::SetAxis(int _index, const math::Vector3 &_axis)
{
  if (_index < 0 || _index > static_cast<int>(this->axisXSpinBoxes.size()))
  {
    gzerr << "Axis index is out of range" << std::endl;
    return;
  }

  this->axisXSpinBoxes[_index]->setValue(_axis.x);
  this->axisYSpinBoxes[_index]->setValue(_axis.y);
  this->axisZSpinBoxes[_index]->setValue(_axis.z);
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
