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

#include "gazebo/gui/building/WallInspectorDialog.hh"
#include "gazebo/gui/building/WallInspectorDialogPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WallInspectorDialog::WallInspectorDialog(QWidget *_parent)
  : BaseInspectorDialog(_parent), dataPtr(new WallInspectorDialogPrivate)
{
  this->setObjectName("wallInspectorDialog");

  this->setWindowTitle(tr("Wall Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QLabel *wallLabel = new QLabel(tr("Wall Name: "));
  this->dataPtr->wallNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(wallLabel);
  nameLayout->addWidget(this->dataPtr->wallNameLabel);

  QLabel *startLabel = new QLabel(tr("Start Point"));
  QLabel *endLabel = new QLabel(tr("End Point"));
  QHBoxLayout *startEndLayout = new QHBoxLayout;
  startEndLayout->addWidget(startLabel);
  startEndLayout->addWidget(endLabel);

  QLabel *startXLabel = new QLabel(tr("x: "));
  QLabel *startYLabel = new QLabel(tr("y: "));

  this->dataPtr->startXSpinBox = new QDoubleSpinBox;
  this->dataPtr->startXSpinBox->setRange(-1000, 1000);
  this->dataPtr->startXSpinBox->setSingleStep(0.001);
  this->dataPtr->startXSpinBox->setDecimals(3);
  this->dataPtr->startXSpinBox->setValue(0.000);
  this->dataPtr->startXSpinBox->setAlignment(Qt::AlignRight);
  QLabel *startXUnitLabel = new QLabel(tr("m "));
  startXUnitLabel->setMaximumWidth(40);

  this->dataPtr->startYSpinBox = new QDoubleSpinBox;
  this->dataPtr->startYSpinBox->setRange(-1000, 1000);
  this->dataPtr->startYSpinBox->setSingleStep(0.001);
  this->dataPtr->startYSpinBox->setDecimals(3);
  this->dataPtr->startYSpinBox->setValue(0.000);
  this->dataPtr->startYSpinBox->setAlignment(Qt::AlignRight);
  QLabel *startYUnitLabel = new QLabel(tr("m "));
  startYUnitLabel->setMaximumWidth(40);

  QLabel *endXLabel = new QLabel(tr("x: "));
  QLabel *endYLabel = new QLabel(tr("y: "));

  this->dataPtr->endXSpinBox = new QDoubleSpinBox;
  this->dataPtr->endXSpinBox->setRange(-1000, 1000);
  this->dataPtr->endXSpinBox->setSingleStep(0.001);
  this->dataPtr->endXSpinBox->setDecimals(3);
  this->dataPtr->endXSpinBox->setValue(0.000);
  this->dataPtr->endXSpinBox->setAlignment(Qt::AlignRight);
  QLabel *endXUnitLabel = new QLabel(tr("m"));
  endXUnitLabel->setMaximumWidth(40);

  this->dataPtr->endYSpinBox = new QDoubleSpinBox;
  this->dataPtr->endYSpinBox->setRange(-1000, 1000);
  this->dataPtr->endYSpinBox->setSingleStep(0.001);
  this->dataPtr->endYSpinBox->setDecimals(3);
  this->dataPtr->endYSpinBox->setValue(0.000);
  this->dataPtr->endYSpinBox->setAlignment(Qt::AlignRight);
  QLabel *endYUnitLabel = new QLabel(tr("m"));
  endYUnitLabel->setMaximumWidth(40);

  QGridLayout *startXYLayout = new QGridLayout;
  startXYLayout->addWidget(startXLabel, 0, 0);
  startXYLayout->addWidget(this->dataPtr->startXSpinBox, 0, 1);
  startXYLayout->addWidget(startXUnitLabel, 0, 2);
  startXYLayout->addWidget(startYLabel, 1, 0);
  startXYLayout->addWidget(this->dataPtr->startYSpinBox, 1, 1);
  startXYLayout->addWidget(startYUnitLabel, 1, 2);
  startXYLayout->setColumnStretch(1, 1);
  startXYLayout->setAlignment(this->dataPtr->startXSpinBox, Qt::AlignLeft);
  startXYLayout->setAlignment(this->dataPtr->startYSpinBox, Qt::AlignLeft);

  QGridLayout *endXYLayout = new QGridLayout;
  endXYLayout->addWidget(endXLabel, 0, 0);
  endXYLayout->addWidget(this->dataPtr->endXSpinBox, 0, 1);
  endXYLayout->addWidget(endXUnitLabel, 0, 2);
  endXYLayout->addWidget(endYLabel, 1, 0);
  endXYLayout->addWidget(this->dataPtr->endYSpinBox, 1, 1);
  endXYLayout->addWidget(endYUnitLabel, 1, 2);
  endXYLayout->setColumnStretch(1, 1);
  endXYLayout->setAlignment(this->dataPtr->endXSpinBox, Qt::AlignLeft);
  endXYLayout->setAlignment(this->dataPtr->endYSpinBox, Qt::AlignLeft);

  QHBoxLayout *xyLayout = new QHBoxLayout;
  xyLayout->addLayout(startXYLayout);
  xyLayout->addLayout(endXYLayout);

  QVBoxLayout *positionGroupLayout = new QVBoxLayout;
  positionGroupLayout->addLayout(startEndLayout);
  positionGroupLayout->addLayout(xyLayout);

  QGroupBox *positionGroupBox = new QGroupBox(tr("Position"));
  positionGroupBox->setLayout(positionGroupLayout);

  QLabel *lengthLabel = new QLabel(tr("Length: "));
  this->dataPtr->lengthSpinBox = new QDoubleSpinBox;
  this->dataPtr->lengthSpinBox->setRange(0, 1000);
  this->dataPtr->lengthSpinBox->setSingleStep(0.001);
  this->dataPtr->lengthSpinBox->setDecimals(3);
  this->dataPtr->lengthSpinBox->setValue(0.000);
  this->dataPtr->lengthSpinBox->setAlignment(Qt::AlignRight);

  QLabel *lengthUnitLabel = new QLabel(tr("m"));
  lengthUnitLabel->setMaximumWidth(40);

  QHBoxLayout *lengthLayout = new QHBoxLayout;
  lengthLayout->addWidget(lengthLabel);
  lengthLayout->addWidget(this->dataPtr->lengthSpinBox);
  lengthLayout->addWidget(lengthUnitLabel);

  QLabel *heightLabel = new QLabel(tr("Height: "));
  this->dataPtr->heightSpinBox = new QDoubleSpinBox;
  this->dataPtr->heightSpinBox->setRange(0, 1000);
  this->dataPtr->heightSpinBox->setSingleStep(0.001);
  this->dataPtr->heightSpinBox->setDecimals(3);
  this->dataPtr->heightSpinBox->setValue(0.000);
  this->dataPtr->heightSpinBox->setAlignment(Qt::AlignRight);

  QLabel *heightUnitLabel = new QLabel(tr("m"));
  heightUnitLabel->setMaximumWidth(40);

  QLabel *thicknessLabel = new QLabel(tr("Thickness "));
  this->dataPtr->thicknessSpinBox = new QDoubleSpinBox;
  this->dataPtr->thicknessSpinBox->setRange(0, 1000);
  this->dataPtr->thicknessSpinBox->setSingleStep(0.001);
  this->dataPtr->thicknessSpinBox->setDecimals(3);
  this->dataPtr->thicknessSpinBox->setValue(0.000);
  this->dataPtr->thicknessSpinBox->setAlignment(Qt::AlignRight);

  QLabel *thicknessUnitLabel = new QLabel(tr("m"));
  thicknessUnitLabel->setMaximumWidth(40);

  QGridLayout *heightThicknessLayout = new QGridLayout;
  heightThicknessLayout->addWidget(heightLabel, 0, 0);
  heightThicknessLayout->addWidget(this->dataPtr->heightSpinBox, 0, 1);
  heightThicknessLayout->addWidget(heightUnitLabel, 0, 2);
  heightThicknessLayout->addWidget(thicknessLabel, 1, 0);
  heightThicknessLayout->addWidget(this->dataPtr->thicknessSpinBox, 1, 1);
  heightThicknessLayout->addWidget(thicknessUnitLabel, 1, 2);

  // TODO Color and texture code is repeated on all dialogs.
  // Make a generalized widget
  this->InitColorComboBox();
  QHBoxLayout *colorLayout = new QHBoxLayout;
  QLabel *colorLabel = new QLabel(tr("Color: "));
  QLabel *colorDummyLabel = new QLabel(tr(""));
  colorDummyLabel->setMaximumWidth(40);
  colorLayout->addWidget(colorLabel);
  colorLayout->addWidget(this->colorComboBox);
  colorLayout->addWidget(colorDummyLabel);

  this->InitTextureComboBox();
  QHBoxLayout *textureLayout = new QHBoxLayout;
  QLabel *textureLabel = new QLabel(tr("Texture: "));
  QLabel *textureDummyLabel = new QLabel(tr(""));
  textureDummyLabel->setMaximumWidth(40);
  textureLayout->addWidget(textureLabel);
  textureLayout->addWidget(this->textureComboBox);
  textureLayout->addWidget(textureDummyLabel);

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
  mainLayout->addLayout(lengthLayout);
  mainLayout->addLayout(heightThicknessLayout);
  mainLayout->addLayout(colorLayout);
  mainLayout->addLayout(textureLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
WallInspectorDialog::~WallInspectorDialog()
{
}

/////////////////////////////////////////////////
double WallInspectorDialog::Length() const
{
  return this->dataPtr->lengthSpinBox->value();
}

/////////////////////////////////////////////////
ignition::math::Vector2d WallInspectorDialog::StartPosition() const
{
  return ignition::math::Vector2d(this->dataPtr->startXSpinBox->value(),
      this->dataPtr->startYSpinBox->value());
}

/////////////////////////////////////////////////
ignition::math::Vector2d WallInspectorDialog::EndPosition() const
{
  return ignition::math::Vector2d(this->dataPtr->endXSpinBox->value(),
      this->dataPtr->endYSpinBox->value());
}

/////////////////////////////////////////////////
double WallInspectorDialog::Height() const
{
  return this->dataPtr->heightSpinBox->value();
}

/////////////////////////////////////////////////
double WallInspectorDialog::Thickness() const
{
  return this->dataPtr->thicknessSpinBox->value();
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetName(const std::string &_name)
{
  this->dataPtr->wallNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetLength(const double _length)
{
  this->dataPtr->lengthSpinBox->setValue(_length);
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetStartPosition(const ignition::math::Vector2d &_pos)
{
  this->dataPtr->startXSpinBox->setValue(_pos.X());
  this->dataPtr->startYSpinBox->setValue(_pos.Y());
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetEndPosition(const ignition::math::Vector2d &_pos)
{
  this->dataPtr->endXSpinBox->setValue(_pos.X());
  this->dataPtr->endYSpinBox->setValue(_pos.Y());
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetHeight(const double _height)
{
  this->dataPtr->heightSpinBox->setValue(_height);
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetThickness(const double _thickness)
{
  this->dataPtr->thicknessSpinBox->setValue(_thickness);
}
