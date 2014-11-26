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

#include "gazebo/gui/building/StairsInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
StairsInspectorDialog::StairsInspectorDialog(QWidget *_parent)
  : QDialog(_parent)
{
  this->setObjectName("stairsInspectorDialog");

  this->setWindowTitle(tr("Stairs Inspector"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  QLabel *stairsLabel = new QLabel(tr("Stairs Name: "));
  this->stairsNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(stairsLabel);
  nameLayout->addWidget(stairsNameLabel);

  QLabel *startPointLabel = new QLabel(tr("Start Point: "));

  QLabel *startXLabel = new QLabel(tr("x: "));
  QLabel *startYLabel = new QLabel(tr("y: "));

  this->startXSpinBox = new QDoubleSpinBox;
  this->startXSpinBox->setRange(-1000, 1000);
  this->startXSpinBox->setSingleStep(0.001);
  this->startXSpinBox->setDecimals(3);
  this->startXSpinBox->setValue(0.000);

  this->startYSpinBox = new QDoubleSpinBox;
  this->startYSpinBox->setRange(-1000, 1000);
  this->startYSpinBox->setSingleStep(0.001);
  this->startYSpinBox->setDecimals(3);
  this->startYSpinBox->setValue(0.000);

  QGridLayout *startXYLayout = new QGridLayout;
  startXYLayout->addWidget(startXLabel, 0, 0);
  startXYLayout->addWidget(startXSpinBox, 0, 1);
  startXYLayout->addWidget(startYLabel, 1, 0);
  startXYLayout->addWidget(startYSpinBox, 1, 1);
  startXYLayout->setColumnStretch(1, 1);
  startXYLayout->setAlignment(startXSpinBox, Qt::AlignLeft);
  startXYLayout->setAlignment(startYSpinBox, Qt::AlignLeft);

  QVBoxLayout *xyLayout = new QVBoxLayout;
  xyLayout->addWidget(startPointLabel);
  xyLayout->addLayout(startXYLayout);

  QGroupBox *positionGroupBox = new QGroupBox(tr("Position"));
  positionGroupBox->setLayout(xyLayout);

  QLabel *widthLabel = new QLabel(tr("Width: "));
  QLabel *depthLabel = new QLabel(tr("Depth: "));
  QLabel *heightLabel = new QLabel(tr("Height: "));

  this->widthSpinBox = new QDoubleSpinBox;
  this->widthSpinBox->setRange(-1000, 1000);
  this->widthSpinBox->setSingleStep(0.001);
  this->widthSpinBox->setDecimals(3);
  this->widthSpinBox->setValue(0.000);

  this->depthSpinBox = new QDoubleSpinBox;
  this->depthSpinBox->setRange(-1000, 1000);
  this->depthSpinBox->setSingleStep(0.001);
  this->depthSpinBox->setDecimals(3);
  this->depthSpinBox->setValue(0.000);

  this->heightSpinBox = new QDoubleSpinBox;
  this->heightSpinBox->setRange(-1000, 1000);
  this->heightSpinBox->setSingleStep(0.001);
  this->heightSpinBox->setDecimals(3);
  this->heightSpinBox->setValue(0.000);

  QGridLayout *sizeLayout = new QGridLayout;
  sizeLayout->addWidget(widthLabel, 0, 0);
  sizeLayout->addWidget(widthSpinBox, 0, 1);
  sizeLayout->addWidget(depthLabel, 1, 0);
  sizeLayout->addWidget(depthSpinBox, 1, 1);
  sizeLayout->addWidget(heightLabel, 2, 0);
  sizeLayout->addWidget(heightSpinBox, 2, 1);

  QLabel *stepsLabel = new QLabel(tr("# Steps: "));
  this->stepsSpinBox = new QSpinBox;
  this->stepsSpinBox->setRange(1, 1000);
  this->stepsSpinBox->setSingleStep(1);
  this->stepsSpinBox->setValue(1);

  QGridLayout *stepsLayout = new QGridLayout;
  stepsLayout->addWidget(stepsLabel, 0, 0);
  stepsLayout->addWidget(stepsSpinBox, 0, 1);

  QVBoxLayout *sizeStepsLayout = new QVBoxLayout;
  sizeStepsLayout->addLayout(sizeLayout);
  sizeStepsLayout->addLayout(stepsLayout);

  QGroupBox *sizeGroupBox = new QGroupBox(tr("Size"));
  sizeGroupBox->setLayout(sizeStepsLayout);

  QLabel *colorLabel = new QLabel(tr("Color: "));
  this->colorComboBox = new QComboBox;
  this->colorComboBox->setIconSize(QSize(15, 15));
  this->colorComboBox->setMinimumWidth(50);
  this->colorComboBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  QPixmap colorIcon(15, 15);
  this->colorList.push_back(QColor(255, 255, 255, 255));
  this->colorList.push_back(QColor(194, 169, 160, 255));
  this->colorList.push_back(QColor(235, 206, 157, 255));
  this->colorList.push_back(QColor(254, 121,   5, 255));
  this->colorList.push_back(QColor(255, 195,  78, 255));
  this->colorList.push_back(QColor(111, 203, 172, 255));
  for (unsigned int i = 0; i < this->colorList.size(); ++i)
  {
    colorIcon.fill(this->colorList.at(i));
    this->colorComboBox->addItem(colorIcon, QString(""));
  }

  QHBoxLayout *colorLayout = new QHBoxLayout;
  colorLayout->addWidget(colorLabel);
  colorLayout->addWidget(colorComboBox);

  QLabel *textureLabel = new QLabel(tr("Texture: "));
  this->textureComboBox = new QComboBox;
  this->textureComboBox->setIconSize(QSize(30, 30));
  this->textureComboBox->setMinimumWidth(50);
  this->textureComboBox->setMinimumHeight(50);
  this->textureComboBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  this->textureList.push_back(":/images/wood.jpg");
  this->textureList.push_back(":/images/ceiling_tiled.jpg");
  for (unsigned int i = 0; i < this->textureList.size(); ++i)
  {
    this->textureComboBox->addItem(QPixmap(this->textureList[i]),
        QString(""));
  }
  this->textureComboBox->addItem("X");
  this->textureComboBox->setCurrentIndex(this->textureComboBox->count()-1);

  QHBoxLayout *textureLayout = new QHBoxLayout;
  textureLayout->addWidget(textureLabel);
  textureLayout->addWidget(textureComboBox);

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
QPointF StairsInspectorDialog::GetStartPosition() const
{
  return QPointF(this->startXSpinBox->value(),
      this->startYSpinBox->value());
}

/////////////////////////////////////////////////
double StairsInspectorDialog::GetWidth() const
{
  return this->widthSpinBox->value();
}

/////////////////////////////////////////////////
double StairsInspectorDialog::GetDepth() const
{
  return this->depthSpinBox->value();
}

/////////////////////////////////////////////////
double StairsInspectorDialog::GetHeight() const
{
  return this->heightSpinBox->value();
}

/////////////////////////////////////////////////
int StairsInspectorDialog::GetSteps() const
{
  return this->stepsSpinBox->value();
}

/////////////////////////////////////////////////
QColor StairsInspectorDialog::GetColor() const
{
  return this->colorList[this->colorComboBox->currentIndex()];
}

/////////////////////////////////////////////////
QString StairsInspectorDialog::GetTexture() const
{
  QString texture = QString("");
  if (this->textureComboBox->currentIndex() != -1 &&
      this->textureComboBox->currentIndex() <
      this->textureComboBox->count() - 1)
  {
    texture = this->textureList[this->textureComboBox->currentIndex()];
  }
  return texture;
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetName(const std::string &_name)
{
  this->stairsNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetStartPosition(const QPointF &_pos)
{
  this->startXSpinBox->setValue(_pos.x());
  this->startYSpinBox->setValue(_pos.y());
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetWidth(double _width)
{
  this->widthSpinBox->setValue(_width);
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetDepth(double _depth)
{
  this->depthSpinBox->setValue(_depth);
}


/////////////////////////////////////////////////
void StairsInspectorDialog::SetHeight(double _height)
{
  this->heightSpinBox->setValue(_height);
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetSteps(int _steps)
{
  this->stepsSpinBox->setValue(_steps);
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetColor(const QColor _color)
{
  // Find index corresponding to color (only a few colors allowed so far)
  for (unsigned int i = 0; i < this->colorList.size(); ++i)
  {
    if (this->colorList[i] == _color)
    {
      this->colorComboBox->setCurrentIndex(i);
      break;
    }
  }
}

/////////////////////////////////////////////////
void StairsInspectorDialog::SetTexture(QString _texture)
{
  // Find index corresponding to texture (only a few textures allowed so far)
  int index = this->textureComboBox->count()-1;
  for (unsigned int i = 0; i < this->textureList.size(); ++i)
  {
    if (this->textureList[i] == _texture)
    {
      index = i;
      break;
    }
  }
  this->textureComboBox->setCurrentIndex(index);
}

/////////////////////////////////////////////////
void StairsInspectorDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void StairsInspectorDialog::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void StairsInspectorDialog::OnOK()
{
  emit Applied();
  this->accept();
}
