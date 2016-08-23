/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"

#include "gazebo/gui/building/StairsInspectorDialog.hh"
#include "gazebo/gui/building/StairsInspectorDialogPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
StairsInspectorDialog::StairsInspectorDialog(QWidget *_parent)
  : BaseInspectorDialog(_parent), dataPtr(new StairsInspectorDialogPrivate)
{
  this->setObjectName("stairsInspectorDialog");

  this->setWindowTitle(tr("Stairs Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QLabel *stairsLabel = new QLabel(tr("Stairs Name: "));
  this->dataPtr->stairsNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(stairsLabel);
  nameLayout->addWidget(this->dataPtr->stairsNameLabel);

  QLabel *startXLabel = new QLabel(tr("x: "));
  QLabel *startYLabel = new QLabel(tr("y: "));

  this->dataPtr->startXSpinBox = new QDoubleSpinBox;
  this->dataPtr->startXSpinBox->setRange(-1000, 1000);
  this->dataPtr->startXSpinBox->setSingleStep(0.001);
  this->dataPtr->startXSpinBox->setDecimals(3);
  this->dataPtr->startXSpinBox->setValue(0.000);
  this->dataPtr->startXSpinBox->setAlignment(Qt::AlignRight);
  QLabel *startXUnitLabel = new QLabel(tr("m"));
  startXUnitLabel->setMaximumWidth(40);

  this->dataPtr->startYSpinBox = new QDoubleSpinBox;
  this->dataPtr->startYSpinBox->setRange(-1000, 1000);
  this->dataPtr->startYSpinBox->setSingleStep(0.001);
  this->dataPtr->startYSpinBox->setDecimals(3);
  this->dataPtr->startYSpinBox->setValue(0.000);
  this->dataPtr->startYSpinBox->setAlignment(Qt::AlignRight);
  QLabel *startYUnitLabel = new QLabel(tr("m"));
  startYUnitLabel->setMaximumWidth(40);

  QGridLayout *startXYLayout = new QGridLayout;
  startXYLayout->addWidget(startXLabel, 0, 0);
  startXYLayout->addWidget(this->dataPtr->startXSpinBox, 0, 1);
  startXYLayout->addWidget(startXUnitLabel, 0, 2);
  startXYLayout->addWidget(startYLabel, 1, 0);
  startXYLayout->addWidget(this->dataPtr->startYSpinBox, 1, 1);
  startXYLayout->addWidget(startYUnitLabel, 1, 2);

  QGroupBox *positionGroupBox = new QGroupBox(tr("Position"));
  positionGroupBox->setLayout(startXYLayout);

  QLabel *widthLabel = new QLabel(tr("Width: "));
  QLabel *depthLabel = new QLabel(tr("Depth: "));
  QLabel *heightLabel = new QLabel(tr("Height: "));

  this->dataPtr->widthSpinBox = new QDoubleSpinBox;
  this->dataPtr->widthSpinBox->setRange(0, 1000);
  this->dataPtr->widthSpinBox->setSingleStep(0.001);
  this->dataPtr->widthSpinBox->setDecimals(3);
  this->dataPtr->widthSpinBox->setValue(0.000);
  this->dataPtr->widthSpinBox->setAlignment(Qt::AlignRight);
  QLabel *widthUnitLabel = new QLabel(tr("m"));
  widthUnitLabel->setMaximumWidth(40);

  this->dataPtr->depthSpinBox = new QDoubleSpinBox;
  this->dataPtr->depthSpinBox->setRange(0, 1000);
  this->dataPtr->depthSpinBox->setSingleStep(0.001);
  this->dataPtr->depthSpinBox->setDecimals(3);
  this->dataPtr->depthSpinBox->setValue(0.000);
  this->dataPtr->depthSpinBox->setAlignment(Qt::AlignRight);
  QLabel *depthUnitLabel = new QLabel(tr("m"));
  depthUnitLabel->setMaximumWidth(40);

  this->dataPtr->heightSpinBox = new QDoubleSpinBox;
  this->dataPtr->heightSpinBox->setRange(0, 1000);
  this->dataPtr->heightSpinBox->setSingleStep(0.001);
  this->dataPtr->heightSpinBox->setDecimals(3);
  this->dataPtr->heightSpinBox->setValue(0.000);
  this->dataPtr->heightSpinBox->setAlignment(Qt::AlignRight);
  QLabel *heightUnitLabel = new QLabel(tr("m"));
  heightUnitLabel->setMaximumWidth(40);

  QLabel *stepsLabel = new QLabel(tr("# Steps: "));
  this->dataPtr->stepsSpinBox = new QSpinBox;
  this->dataPtr->stepsSpinBox->setRange(1, 1000);
  this->dataPtr->stepsSpinBox->setSingleStep(1);
  this->dataPtr->stepsSpinBox->setValue(1);
  this->dataPtr->stepsSpinBox->setAlignment(Qt::AlignRight);
  QLabel *stepsDummyLabel = new QLabel(tr(" "));

  QGridLayout *sizeLayout = new QGridLayout;
  sizeLayout->addWidget(widthLabel, 0, 0);
  sizeLayout->addWidget(this->dataPtr->widthSpinBox, 0, 1);
  sizeLayout->addWidget(widthUnitLabel, 0, 2);
  sizeLayout->addWidget(depthLabel, 1, 0);
  sizeLayout->addWidget(this->dataPtr->depthSpinBox, 1, 1);
  sizeLayout->addWidget(depthUnitLabel, 1, 2);
  sizeLayout->addWidget(heightLabel, 2, 0);
  sizeLayout->addWidget(this->dataPtr->heightSpinBox, 2, 1);
  sizeLayout->addWidget(heightUnitLabel, 2, 2);
  sizeLayout->addWidget(stepsLabel, 3, 0);
  sizeLayout->addWidget(this->dataPtr->stepsSpinBox, 3, 1);
  sizeLayout->addWidget(stepsDummyLabel, 3, 2);

  QGroupBox *sizeGroupBox = new QGroupBox(tr("Size"));
  sizeGroupBox->setLayout(sizeLayout);

  this->InitColorComboBox();
  QHBoxLayout *colorLayout = new QHBoxLayout;
  QLabel *colorLabel = new QLabel(tr("Color: "));
  colorLayout->addWidget(colorLabel);
  colorLayout->addWidget(this->colorComboBox);

  this->InitTextureComboBox();
  QHBoxLayout *textureLayout = new QHBoxLayout;
  QLabel *textureLabel = new QLabel(tr("Texture: "));
  textureLayout->addWidget(textureLabel);
  textureLayout->addWidget(this->textureComboBox);

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
  mainLayout->addWidget(positionGroupBox);
  mainLayout->addWidget(sizeGroupBox);
  mainLayout->addLayout(colorLayout);
  mainLayout->addLayout(textureLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
StairsInspectorDialog::~StairsInspectorDialog()
{
}

/////////////////////////////////////////////////
ignition::math::Vector2d StairsInspectorDialog::StartPosition() const
{
  return ignition::math::Vector2d(
      this->dataPtr->startXSpinBox->value(),
      this->dataPtr->startYSpinBox->value());
}

/////////////////////////////////////////////////
double StairsInspectorDialog::Width() const
{
  return this->dataPtr->widthSpinBox->value();
}

/////////////////////////////////////////////////
double StairsInspectorDialog::Depth() const
{
  return this->dataPtr->depthSpinBox->value();
}

/////////////////////////////////////////////////
double StairsInspectorDialog::Height() const
{
  return this->dataPtr->heightSpinBox->value();
}

/////////////////////////////////////////////////
int StairsInspectorDialog::Steps() const
{
  return this->dataPtr->stepsSpinBox->value();
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetName(const std::string &_name)
{
  this->dataPtr->stairsNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetStartPosition(
    const ignition::math::Vector2d &_pos)
{
  this->dataPtr->startXSpinBox->setValue(_pos.X());
  this->dataPtr->startYSpinBox->setValue(_pos.Y());
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetWidth(const double _width)
{
  this->dataPtr->widthSpinBox->setValue(_width);
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetDepth(const double _depth)
{
  this->dataPtr->depthSpinBox->setValue(_depth);
}


/////////////////////////////////////////////////
void StairsInspectorDialog::SetHeight(const double _height)
{
  this->dataPtr->heightSpinBox->setValue(_height);
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetSteps(int _steps)
{
  this->dataPtr->stepsSpinBox->setValue(_steps);
}
