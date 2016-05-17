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

#include "gazebo/gui/building/LevelInspectorDialog.hh"
#include "gazebo/gui/building/LevelInspectorDialogPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LevelInspectorDialog::LevelInspectorDialog(QWidget *_parent)
  : BaseInspectorDialog(_parent), dataPtr(new LevelInspectorDialogPrivate)
{
  this->setObjectName("levelInspectorDialog");
  this->setWindowTitle(tr("Level Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QLabel *levelLabel = new QLabel(tr("Level Name: "));
  this->dataPtr->levelNameLineEdit = new QLineEdit;
  this->dataPtr->levelNameLineEdit->setPlaceholderText(tr("Level X"));

  this->InitColorComboBox();
  QHBoxLayout *colorLayout = new QHBoxLayout;
  QLabel *colorLabel = new QLabel(tr("Floor Color: "));
  colorLayout->addWidget(colorLabel);
  colorLayout->addWidget(this->colorComboBox);

  this->InitTextureComboBox();
  QHBoxLayout *textureLayout = new QHBoxLayout;
  QLabel *textureLabel = new QLabel(tr("Floor Texture: "));
  textureLayout->addWidget(textureLabel);
  textureLayout->addWidget(this->textureComboBox);

  QVBoxLayout *floorLayout = new QVBoxLayout;
  floorLayout->addLayout(colorLayout);
  floorLayout->addLayout(textureLayout);

  this->dataPtr->floorWidget = new QWidget;
  this->dataPtr->floorWidget->setLayout(floorLayout);

  /// TODO add the widgets back in after the functions is implemented
/*  QLabel *floorThicknessLabel = new QLabel(tr("Floor Thickness: "));
  this->dataPtr->floorThicknessSpinBox = new QDoubleSpinBox;
  this->dataPtr->floorThicknessSpinBox->setRange(-1000, 1000);
  this->dataPtr->floorThicknessSpinBox->setSingleStep(0.001);
  this->dataPtr->floorThicknessSpinBox->setDecimals(3);
  this->dataPtr->floorThicknessSpinBox->setValue(0.000);

  QLabel *heightLabel = new QLabel(tr("Height: "));
  this->dataPtr->heightSpinBox = new QDoubleSpinBox;
  this->dataPtr->heightSpinBox->setRange(-1000, 1000);
  this->dataPtr->heightSpinBox->setSingleStep(0.001);
  this->dataPtr->heightSpinBox->setDecimals(3);
  this->dataPtr->heightSpinBox->setValue(0.000);*/

  QGridLayout *levelLayout = new QGridLayout;
  levelLayout->addWidget(levelLabel, 0, 0);
  levelLayout->addWidget(this->dataPtr->levelNameLineEdit, 0, 1);
/*  levelLayout->addWidget(floorThicknessLabel, 1, 0);
  levelLayout->addWidget(this->dataPtr->floorThicknessSpinBox, 1, 1);
  levelLayout->addWidget(heightLabel, 2, 0);
  levelLayout->addWidget(this->dataPtr->heightSpinBox, 2, 1);*/

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
  mainLayout->addLayout(levelLayout);
  mainLayout->addWidget(this->dataPtr->floorWidget);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
LevelInspectorDialog::~LevelInspectorDialog()
{
}

/////////////////////////////////////////////////
std::string LevelInspectorDialog::LevelName() const
{
  return this->dataPtr->levelNameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
double LevelInspectorDialog::Height() const
{
  return this->dataPtr->heightSpinBox->value();
}

/////////////////////////////////////////////////
void LevelInspectorDialog::SetLevelName(const std::string &_levelName)
{
  this->dataPtr->levelNameLineEdit->setText(QString(_levelName.c_str()));
}


/////////////////////////////////////////////////
void LevelInspectorDialog::SetHeight(const double _height)
{
  this->dataPtr->heightSpinBox->setValue(_height);
}

/////////////////////////////////////////////////
void LevelInspectorDialog::ShowFloorWidget(const bool _show)
{
  if (_show)
    this->dataPtr->floorWidget->show();
  else
    this->dataPtr->floorWidget->hide();
}
