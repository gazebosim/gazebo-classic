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

#include "gazebo/gui/building/WindowDoorInspectorDialog.hh"
#include "gazebo/gui/building/WindowDoorInspectorDialogPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WindowDoorInspectorDialog::WindowDoorInspectorDialog(int _mode,
  QWidget *_parent)
  : BaseInspectorDialog(_parent), dataPtr(new WindowDoorInspectorDialogPrivate)
{
  this->setObjectName("windowDoorInspectorDialog");

  int dialogMode = _mode;

  std::string dialogModeStr = "";
  if (dialogMode == WINDOW)
    dialogModeStr = "Window";
  else if (dialogMode == DOOR)
    dialogModeStr = "Door";

  std::string titleStr = dialogModeStr + " Inspector";
  this->setWindowTitle(tr(titleStr.c_str()));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  std::string itemLabelStr = dialogModeStr + " Name:";
  QLabel *itemlLabel = new QLabel(tr(itemLabelStr.c_str()));
  this->dataPtr->itemNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(itemlLabel);
  nameLayout->addWidget(this->dataPtr->itemNameLabel);

  QLabel *widthLabel = new QLabel(tr("Width: "));
//  QLabel *depthLabel = new QLabel(tr("Depth: "));
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

  this->dataPtr->heightSpinBox = new QDoubleSpinBox;
  this->dataPtr->heightSpinBox->setRange(0, 1000);
  this->dataPtr->heightSpinBox->setSingleStep(0.001);
  this->dataPtr->heightSpinBox->setDecimals(3);
  this->dataPtr->heightSpinBox->setValue(0.000);
  this->dataPtr->heightSpinBox->setAlignment(Qt::AlignRight);
  QLabel *heightUnitLabel = new QLabel(tr("m"));
  heightUnitLabel->setMaximumWidth(40);

  QGridLayout *sizeLayout = new QGridLayout;
  sizeLayout->addWidget(widthLabel, 0, 0);
  sizeLayout->addWidget(this->dataPtr->widthSpinBox, 0, 1);
  sizeLayout->addWidget(widthUnitLabel, 0, 2);
//  sizeLayout->addWidget(depthLabel, 1, 0);
//  sizeLayout->addWidget(this->dataPtr->depthSpinBox, 1, 1);
  sizeLayout->addWidget(heightLabel, 2, 0);
  sizeLayout->addWidget(this->dataPtr->heightSpinBox, 2, 1);
  sizeLayout->addWidget(heightUnitLabel, 2, 2);

  QGroupBox *sizeGroupBox = new QGroupBox(tr("Size"));
  sizeGroupBox->setLayout(sizeLayout);

  QLabel *positionXLabel = new QLabel(tr("x: "));
  QLabel *positionYLabel = new QLabel(tr("y: "));
  QLabel *elevationLabel = new QLabel(tr("Elevation: "));

  this->dataPtr->positionXSpinBox = new QDoubleSpinBox;
  this->dataPtr->positionXSpinBox->setRange(-1000, 1000);
  this->dataPtr->positionXSpinBox->setSingleStep(0.001);
  this->dataPtr->positionXSpinBox->setDecimals(3);
  this->dataPtr->positionXSpinBox->setValue(0.000);
  this->dataPtr->positionXSpinBox->setAlignment(Qt::AlignRight);
  QLabel *positionXUnitLabel = new QLabel(tr("m"));
  positionXUnitLabel->setMaximumWidth(40);

  this->dataPtr->positionYSpinBox = new QDoubleSpinBox;
  this->dataPtr->positionYSpinBox->setRange(-1000, 1000);
  this->dataPtr->positionYSpinBox->setSingleStep(0.001);
  this->dataPtr->positionYSpinBox->setDecimals(3);
  this->dataPtr->positionYSpinBox->setValue(0.000);
  this->dataPtr->positionYSpinBox->setAlignment(Qt::AlignRight);
  QLabel *positionYUnitLabel = new QLabel(tr("m"));
  positionYUnitLabel->setMaximumWidth(40);

  this->dataPtr->elevationSpinBox = new QDoubleSpinBox;
  this->dataPtr->elevationSpinBox->setRange(0, 1000);
  this->dataPtr->elevationSpinBox->setSingleStep(0.001);
  this->dataPtr->elevationSpinBox->setDecimals(3);
  this->dataPtr->elevationSpinBox->setValue(0.000);
  this->dataPtr->elevationSpinBox->setAlignment(Qt::AlignRight);
  QLabel *elevationUnitLabel = new QLabel(tr("m"));
  elevationUnitLabel->setMaximumWidth(40);

  QGridLayout *positionLayout = new QGridLayout;
  positionLayout->addWidget(positionXLabel, 0, 0);
  positionLayout->addWidget(this->dataPtr->positionXSpinBox, 0, 1);
  positionLayout->addWidget(positionXUnitLabel, 0, 2);
  positionLayout->addWidget(positionYLabel), 1, 0;
  positionLayout->addWidget(this->dataPtr->positionYSpinBox, 1, 1);
  positionLayout->addWidget(positionYUnitLabel, 1, 2);
  positionLayout->addWidget(elevationLabel, 0, 3);
  positionLayout->addWidget(this->dataPtr->elevationSpinBox, 0, 4);
  positionLayout->addWidget(elevationUnitLabel, 0, 5);

  QGroupBox *positionGroupBox = new QGroupBox(tr("Position"));
  positionGroupBox->setLayout(positionLayout);

  QLabel *typeLabel = new QLabel(tr("Type: "));
  this->dataPtr->typeComboBox = new QComboBox;
  this->dataPtr->typeComboBox->addItem(QString("Single"));

  QHBoxLayout *typeLayout = new QHBoxLayout;
  typeLayout->addWidget(typeLabel);
  typeLayout->addWidget(this->dataPtr->typeComboBox);

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
  // mainLayout->addLayout(typeLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
WindowDoorInspectorDialog::~WindowDoorInspectorDialog()
{
}

/////////////////////////////////////////////////
double WindowDoorInspectorDialog::Width() const
{
  return this->dataPtr->widthSpinBox->value();
}

/////////////////////////////////////////////////
double WindowDoorInspectorDialog::Depth() const
{
  return this->dataPtr->depthSpinBox->value();
}

/////////////////////////////////////////////////
double WindowDoorInspectorDialog::Height() const
{
  return this->dataPtr->heightSpinBox->value();
}

/////////////////////////////////////////////////
ignition::math::Vector2d WindowDoorInspectorDialog::Position() const
{
  return ignition::math::Vector2d(
      this->dataPtr->positionXSpinBox->value(),
      this->dataPtr->positionYSpinBox->value());
}

/////////////////////////////////////////////////
double WindowDoorInspectorDialog::Elevation() const
{
  return this->dataPtr->elevationSpinBox->value();
}

/////////////////////////////////////////////////
std::string WindowDoorInspectorDialog::Type() const
{
  return this->dataPtr->typeComboBox->currentText().toStdString();
}

/////////////////////////////////////////////////
void WindowDoorInspectorDialog::SetName(const std::string &_name)
{
  this->dataPtr->itemNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void WindowDoorInspectorDialog::SetWidth(const double _width)
{
  this->dataPtr->widthSpinBox->setValue(_width);
}

/////////////////////////////////////////////////
void WindowDoorInspectorDialog::SetHeight(const double _height)
{
  this->dataPtr->heightSpinBox->setValue(_height);
}

/////////////////////////////////////////////////
void WindowDoorInspectorDialog::SetDepth(const double _depth)
{
  this->dataPtr->depthSpinBox->setValue(_depth);
}

/////////////////////////////////////////////////
void WindowDoorInspectorDialog::SetPosition(
    const ignition::math::Vector2d &_pos)
{
  this->dataPtr->positionXSpinBox->setValue(_pos.X());
  this->dataPtr->positionYSpinBox->setValue(_pos.Y());
}

/////////////////////////////////////////////////
void WindowDoorInspectorDialog::SetElevation(const double _elevation)
{
  this->dataPtr->elevationSpinBox->setValue(_elevation);
}

/////////////////////////////////////////////////
void WindowDoorInspectorDialog::SetType(const std::string &_type)
{
  int index = this->dataPtr->typeComboBox->findText(
    QString::fromStdString(_type));
  this->dataPtr->typeComboBox->setCurrentIndex(index);
}
