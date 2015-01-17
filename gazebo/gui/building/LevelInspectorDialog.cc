/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LevelInspectorDialog::LevelInspectorDialog(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("levelInspectorDialog");
  this->setWindowTitle(tr("Level Inspector"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  QLabel *levelLabel = new QLabel(tr("Level Name: "));
  this->levelNameLineEdit = new QLineEdit;
  this->levelNameLineEdit->setPlaceholderText(tr("Level X"));

  QLabel *floorColorLabel = new QLabel(tr("Floor Color: "));
  this->floorColorComboBox = new QComboBox;
  this->floorColorComboBox->setIconSize(QSize(15, 15));
  this->floorColorComboBox->setMinimumWidth(50);
  this->floorColorComboBox->setSizePolicy(QSizePolicy::Fixed,
      QSizePolicy::Fixed);
  QPixmap floorColorIcon(15, 15);
  this->floorColorList.push_back(QColor(255, 255, 255, 255));
  this->floorColorList.push_back(QColor(194, 169, 160, 255));
  this->floorColorList.push_back(QColor(235, 206, 157, 255));
  this->floorColorList.push_back(QColor(254, 121,   5, 255));
  this->floorColorList.push_back(QColor(255, 195,  78, 255));
  this->floorColorList.push_back(QColor(111, 203, 172, 255));
  for (unsigned int i = 0; i < this->floorColorList.size(); ++i)
  {
    floorColorIcon.fill(this->floorColorList.at(i));
    this->floorColorComboBox->addItem(floorColorIcon, QString(""));
  }

  QHBoxLayout *floorColorLayout = new QHBoxLayout;
  floorColorLayout->addWidget(floorColorLabel);
  floorColorLayout->addWidget(floorColorComboBox);

  QLabel *floorTextureLabel = new QLabel(tr("Floor Texture: "));
  this->floorTextureComboBox = new QComboBox;
  this->floorTextureComboBox->setIconSize(QSize(30, 30));
  this->floorTextureComboBox->setMinimumWidth(50);
  this->floorTextureComboBox->setMinimumHeight(50);
  this->floorTextureComboBox->setSizePolicy(QSizePolicy::Fixed,
      QSizePolicy::Fixed);
  this->floorTextureList.push_back(":wood.jpg");
  this->floorTextureList.push_back(":tiles.jpg");
  this->floorTextureList.push_back(":bricks.png");
  for (unsigned int i = 0; i < this->floorTextureList.size(); ++i)
  {
    this->floorTextureComboBox->addItem(QPixmap(this->floorTextureList[i])
        .scaled(QSize(90, 90), Qt::IgnoreAspectRatio), QString(""));
  }
  this->floorTextureComboBox->addItem("X");
  this->floorTextureComboBox->setCurrentIndex(
      this->floorTextureComboBox->count()-1);

  QHBoxLayout *floorTextureLayout = new QHBoxLayout;
  floorTextureLayout->addWidget(floorTextureLabel);
  floorTextureLayout->addWidget(floorTextureComboBox);

  QVBoxLayout *floorLayout = new QVBoxLayout;
  floorLayout->addLayout(floorColorLayout);
  floorLayout->addLayout(floorTextureLayout);

  this->floorWidget = new QWidget;
  this->floorWidget->setLayout(floorLayout);

  /// TODO add the widgets back in after the functions is implemented
/*  QLabel *floorThicknessLabel = new QLabel(tr("Floor Thickness: "));
  this->floorThicknessSpinBox = new QDoubleSpinBox;
  this->floorThicknessSpinBox->setRange(-1000, 1000);
  this->floorThicknessSpinBox->setSingleStep(0.001);
  this->floorThicknessSpinBox->setDecimals(3);
  this->floorThicknessSpinBox->setValue(0.000);

  QLabel *heightLabel = new QLabel(tr("Height: "));
  this->heightSpinBox = new QDoubleSpinBox;
  this->heightSpinBox->setRange(-1000, 1000);
  this->heightSpinBox->setSingleStep(0.001);
  this->heightSpinBox->setDecimals(3);
  this->heightSpinBox->setValue(0.000);*/

  QGridLayout *levelLayout = new QGridLayout;
  levelLayout->addWidget(levelLabel, 0, 0);
  levelLayout->addWidget(this->levelNameLineEdit, 0, 1);
/*  levelLayout->addWidget(floorThicknessLabel, 1, 0);
  levelLayout->addWidget(this->floorThicknessSpinBox, 1, 1);
  levelLayout->addWidget(heightLabel, 2, 0);
  levelLayout->addWidget(this->heightSpinBox, 2, 1);*/

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
  mainLayout->addWidget(this->floorWidget);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
LevelInspectorDialog::~LevelInspectorDialog()
{
}

/////////////////////////////////////////////////
std::string LevelInspectorDialog::GetLevelName() const
{
  return this->levelNameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
double LevelInspectorDialog::GetHeight() const
{
  return this->heightSpinBox->value();
}

/////////////////////////////////////////////////
QColor LevelInspectorDialog::GetFloorColor() const
{
  return this->floorColorList[this->floorColorComboBox->currentIndex()];
}

/////////////////////////////////////////////////
QString LevelInspectorDialog::GetFloorTexture() const
{
  QString floorTexture = QString("");
  if (this->floorTextureComboBox->currentIndex() != -1 &&
      this->floorTextureComboBox->currentIndex() <
      this->floorTextureComboBox->count() - 1)
  {
    floorTexture = this->floorTextureList[
        this->floorTextureComboBox->currentIndex()];
  }
  return floorTexture;
}

/////////////////////////////////////////////////
void LevelInspectorDialog::SetLevelName(const std::string &_levelName)
{
  this->levelNameLineEdit->setText(QString(_levelName.c_str()));
}


/////////////////////////////////////////////////
void LevelInspectorDialog::SetHeight(double _height)
{
  this->heightSpinBox->setValue(_height);
}

/////////////////////////////////////////////////
void LevelInspectorDialog::SetFloorColor(const QColor _color)
{
  int index = -1;
  for (unsigned int i = 0; i < this->floorColorList.size(); ++i)
  {
    if (this->floorColorList[i] == _color)
    {
      index = i;
      break;
    }
  }

  if (index == -1)
  {
    // Add a new color
    this->floorColorList.push_back(_color);
    QPixmap colorIcon(15, 15);
    colorIcon.fill(this->floorColorList.back());
    this->floorColorComboBox->addItem(colorIcon, QString(""));
    index = this->floorColorComboBox->count()-1;
  }
  GZ_ASSERT(index > 0, "Color index is broken < 0");
  this->floorColorComboBox->setCurrentIndex(index);
}

/////////////////////////////////////////////////
void LevelInspectorDialog::SetFloorTexture(QString _floorTexture)
{
  // Find index corresponding to texture (only a few textures allowed so far)
  int index = this->floorTextureComboBox->count()-1;
  for (unsigned int i = 0; i < this->floorTextureList.size(); ++i)
  {
    if (this->floorTextureList[i] == _floorTexture)
    {
      index = i;
      break;
    }
  }
  this->floorTextureComboBox->setCurrentIndex(index);
}

/////////////////////////////////////////////////
void LevelInspectorDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void LevelInspectorDialog::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void LevelInspectorDialog::OnOK()
{
  emit Applied();
  this->accept();
}
