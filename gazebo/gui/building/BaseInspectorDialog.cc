/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/gui/building/BaseInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BaseInspectorDialog::BaseInspectorDialog(QWidget *_parent):QDialog(_parent)
{
}

/////////////////////////////////////////////////
BaseInspectorDialog::~BaseInspectorDialog()
{
}

/////////////////////////////////////////////////
void BaseInspectorDialog::InitColorComboBox()
{
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
}

/////////////////////////////////////////////////
void BaseInspectorDialog::InitTextureComboBox()
{
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
}

/////////////////////////////////////////////////
QColor BaseInspectorDialog::GetColor() const
{
  return this->colorList[this->colorComboBox->currentIndex()];
}

/////////////////////////////////////////////////
QString BaseInspectorDialog::GetTexture() const
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
void BaseInspectorDialog::SetColor(const QColor _color)
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
void BaseInspectorDialog::SetTexture(QString _texture)
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
