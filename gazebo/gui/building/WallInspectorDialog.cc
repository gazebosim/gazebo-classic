/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WallInspectorDialog::WallInspectorDialog(QWidget *_parent)
  : QDialog(_parent)
{
  this->setObjectName("wallInspectorDialog");

  this->setWindowTitle(tr("Wall Inspector"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  QLabel *wallLabel = new QLabel(tr("Wall Name: "));
  this->wallNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(wallLabel);
  nameLayout->addWidget(this->wallNameLabel);

  QLabel *startLabel = new QLabel(tr("Start Point"));
  QLabel *endLabel = new QLabel(tr("End Point"));
  QHBoxLayout *startEndLayout = new QHBoxLayout;
  startEndLayout->addWidget(startLabel);
  startEndLayout->addWidget(endLabel);

  QLabel *startXLabel = new QLabel(tr("x: "));
  QLabel *startYLabel = new QLabel(tr("y: "));

  this->startXSpinBox = new QDoubleSpinBox;
  this->startXSpinBox->setRange(-1000, 1000);
  this->startXSpinBox->setSingleStep(0.001);
  this->startXSpinBox->setDecimals(3);
  this->startXSpinBox->setValue(0.000);
  this->startXSpinBox->setAlignment(Qt::AlignRight);
  QLabel *startXUnitLabel = new QLabel(tr("m "));
  startXUnitLabel->setMaximumWidth(40);

  this->startYSpinBox = new QDoubleSpinBox;
  this->startYSpinBox->setRange(-1000, 1000);
  this->startYSpinBox->setSingleStep(0.001);
  this->startYSpinBox->setDecimals(3);
  this->startYSpinBox->setValue(0.000);
  this->startYSpinBox->setAlignment(Qt::AlignRight);
  QLabel *startYUnitLabel = new QLabel(tr("m "));
  startYUnitLabel->setMaximumWidth(40);

  QLabel *endXLabel = new QLabel(tr("x: "));
  QLabel *endYLabel = new QLabel(tr("y: "));

  this->endXSpinBox = new QDoubleSpinBox;
  this->endXSpinBox->setRange(-1000, 1000);
  this->endXSpinBox->setSingleStep(0.001);
  this->endXSpinBox->setDecimals(3);
  this->endXSpinBox->setValue(0.000);
  this->endXSpinBox->setAlignment(Qt::AlignRight);
  QLabel *endXUnitLabel = new QLabel(tr("m"));
  endXUnitLabel->setMaximumWidth(40);

  this->endYSpinBox = new QDoubleSpinBox;
  this->endYSpinBox->setRange(-1000, 1000);
  this->endYSpinBox->setSingleStep(0.001);
  this->endYSpinBox->setDecimals(3);
  this->endYSpinBox->setValue(0.000);
  this->endYSpinBox->setAlignment(Qt::AlignRight);
  QLabel *endYUnitLabel = new QLabel(tr("m"));
  endYUnitLabel->setMaximumWidth(40);

  QGridLayout *startXYLayout = new QGridLayout;
  startXYLayout->addWidget(startXLabel, 0, 0);
  startXYLayout->addWidget(this->startXSpinBox, 0, 1);
  startXYLayout->addWidget(startXUnitLabel, 0, 2);
  startXYLayout->addWidget(startYLabel, 1, 0);
  startXYLayout->addWidget(this->startYSpinBox, 1, 1);
  startXYLayout->addWidget(startYUnitLabel, 1, 2);
  startXYLayout->setColumnStretch(1, 1);
  startXYLayout->setAlignment(this->startXSpinBox, Qt::AlignLeft);
  startXYLayout->setAlignment(this->startYSpinBox, Qt::AlignLeft);

  QGridLayout *endXYLayout = new QGridLayout;
  endXYLayout->addWidget(endXLabel, 0, 0);
  endXYLayout->addWidget(this->endXSpinBox, 0, 1);
  endXYLayout->addWidget(endXUnitLabel, 0, 2);
  endXYLayout->addWidget(endYLabel, 1, 0);
  endXYLayout->addWidget(this->endYSpinBox, 1, 1);
  endXYLayout->addWidget(endYUnitLabel, 1, 2);
  endXYLayout->setColumnStretch(1, 1);
  endXYLayout->setAlignment(this->endXSpinBox, Qt::AlignLeft);
  endXYLayout->setAlignment(this->endYSpinBox, Qt::AlignLeft);

  QHBoxLayout *xyLayout = new QHBoxLayout;
  xyLayout->addLayout(startXYLayout);
  xyLayout->addLayout(endXYLayout);


  QVBoxLayout *positionGroupLayout = new QVBoxLayout;
  positionGroupLayout->addLayout(startEndLayout);
  positionGroupLayout->addLayout(xyLayout);

  QGroupBox *positionGroupBox = new QGroupBox(tr("Position"));
  positionGroupBox->setLayout(positionGroupLayout);

  QLabel *lengthLabel = new QLabel(tr("Length: "));
  this->lengthSpinBox = new QDoubleSpinBox;
  this->lengthSpinBox->setRange(0, 1000);
  this->lengthSpinBox->setSingleStep(0.001);
  this->lengthSpinBox->setDecimals(3);
  this->lengthSpinBox->setValue(0.000);
  this->lengthSpinBox->setAlignment(Qt::AlignRight);

  QLabel *lengthUnitLabel = new QLabel(tr("m"));
  lengthUnitLabel->setMaximumWidth(40);

  QHBoxLayout *lengthLayout = new QHBoxLayout;
  lengthLayout->addWidget(lengthLabel);
  lengthLayout->addWidget(this->lengthSpinBox);
  lengthLayout->addWidget(lengthUnitLabel);

  QLabel *heightLabel = new QLabel(tr("Height: "));
  this->heightSpinBox = new QDoubleSpinBox;
  this->heightSpinBox->setRange(0, 1000);
  this->heightSpinBox->setSingleStep(0.001);
  this->heightSpinBox->setDecimals(3);
  this->heightSpinBox->setValue(0.000);
  this->heightSpinBox->setAlignment(Qt::AlignRight);

  QLabel *heightUnitLabel = new QLabel(tr("m"));
  heightUnitLabel->setMaximumWidth(40);

  QLabel *thicknessLabel = new QLabel(tr("Thickness "));
  this->thicknessSpinBox = new QDoubleSpinBox;
  this->thicknessSpinBox->setRange(0, 1000);
  this->thicknessSpinBox->setSingleStep(0.001);
  this->thicknessSpinBox->setDecimals(3);
  this->thicknessSpinBox->setValue(0.000);
  this->thicknessSpinBox->setAlignment(Qt::AlignRight);

  QLabel *thicknessUnitLabel = new QLabel(tr("m"));
  thicknessUnitLabel->setMaximumWidth(40);

  QGridLayout *heightThicknessLayout = new QGridLayout;
  heightThicknessLayout->addWidget(heightLabel, 0, 0);
  heightThicknessLayout->addWidget(this->heightSpinBox, 0, 1);
  heightThicknessLayout->addWidget(heightUnitLabel, 0, 2);
  heightThicknessLayout->addWidget(thicknessLabel, 1, 0);
  heightThicknessLayout->addWidget(this->thicknessSpinBox, 1, 1);
  heightThicknessLayout->addWidget(thicknessUnitLabel, 1, 2);

  // TODO Color and texture code is repeated on all dialogs.
  // Make a generalized widget
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

  QLabel *colorDummyLabel = new QLabel(tr(""));
  colorDummyLabel->setMaximumWidth(40);

  QHBoxLayout *colorLayout = new QHBoxLayout;
  colorLayout->addWidget(colorLabel);
  colorLayout->addWidget(this->colorComboBox);
  colorLayout->addWidget(colorDummyLabel);

  QLabel *textureLabel = new QLabel(tr("Texture: "));
  this->textureComboBox = new QComboBox;
  this->textureComboBox->setIconSize(QSize(30, 30));
  this->textureComboBox->setMinimumWidth(50);
  this->textureComboBox->setMinimumHeight(50);
  this->textureComboBox->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  this->textureList.push_back(":wood.jpg");
  this->textureList.push_back(":tiles.jpg");
  this->textureList.push_back(":bricks.png");
  for (unsigned int i = 0; i < this->textureList.size(); ++i)
  {
    this->textureComboBox->addItem(QPixmap(this->textureList[i]).scaled(
        QSize(90, 90), Qt::IgnoreAspectRatio), QString(""));
  }
  this->textureComboBox->addItem("X");
  this->textureComboBox->setCurrentIndex(this->textureComboBox->count()-1);

  QLabel *textureDummyLabel = new QLabel(tr(""));
  textureDummyLabel->setMaximumWidth(40);

  QHBoxLayout *textureLayout = new QHBoxLayout;
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
double WallInspectorDialog::GetLength() const
{
  return this->lengthSpinBox->value();
}

/////////////////////////////////////////////////
QPointF WallInspectorDialog::GetStartPosition() const
{
  return QPointF(this->startXSpinBox->value(),
      this->startYSpinBox->value());
}

/////////////////////////////////////////////////
QPointF WallInspectorDialog::GetEndPosition() const
{
  return QPointF(this->endXSpinBox->value(),
      this->endYSpinBox->value());
}

/////////////////////////////////////////////////
double WallInspectorDialog::GetHeight() const
{
  return this->heightSpinBox->value();
}

/////////////////////////////////////////////////
double WallInspectorDialog::GetThickness() const
{
  return this->thicknessSpinBox->value();
}

/////////////////////////////////////////////////
QColor WallInspectorDialog::GetColor() const
{
  return this->colorList[this->colorComboBox->currentIndex()];
}

/////////////////////////////////////////////////
QString WallInspectorDialog::GetTexture() const
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
void WallInspectorDialog::SetName(const std::string &_name)
{
  this->wallNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetLength(double _length)
{
  this->lengthSpinBox->setValue(_length);
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetStartPosition(const QPointF &_pos)
{
  this->startXSpinBox->setValue(_pos.x());
  this->startYSpinBox->setValue(_pos.y());
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetEndPosition(const QPointF &_pos)
{
  this->endXSpinBox->setValue(_pos.x());
  this->endYSpinBox->setValue(_pos.y());
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetHeight(double _height)
{
  this->heightSpinBox->setValue(_height);
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetThickness(double _thickness)
{
  this->thicknessSpinBox->setValue(_thickness);
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetColor(const QColor _color)
{
  int index = -1;
  for (unsigned int i = 0; i < this->colorList.size(); ++i)
  {
    if (this->colorList[i] == _color)
    {
      index = i;
      break;
    }
  }

  if (index == -1)
  {
    // Add a new color
    this->colorList.push_back(_color);
    QPixmap colorIcon(15, 15);
    colorIcon.fill(this->colorList.back());
    this->colorComboBox->addItem(colorIcon, QString(""));
    index = this->colorComboBox->count()-1;
  }
  GZ_ASSERT(index >= 0, "Color index is broken < 0");
  this->colorComboBox->setCurrentIndex(index);
}

/////////////////////////////////////////////////
void WallInspectorDialog::SetTexture(QString _texture)
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
void WallInspectorDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void WallInspectorDialog::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void WallInspectorDialog::OnOK()
{
  emit Applied();
  this->accept();
}
