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

#include <sstream>
#include "gui/model_editor/LevelWidget.hh"
#include "gui/model_editor/EditorEvents.hh"

using namespace gazebo;
using namespace gui;

//////////////////////////////////////////////////
LevelWidget::LevelWidget(QWidget *_parent) : QWidget(_parent)
{
  this->setObjectName("levelWidget");

  QHBoxLayout *levelLayout = new QHBoxLayout;

  this->levelComboBox = new QComboBox;
  this->levelComboBox->addItem(QString("Level 1"));
  int comboBoxwidth = levelComboBox->minimumSizeHint().width();
  int comboBoxHeight = levelComboBox->minimumSizeHint().height();
  this->levelComboBox->setMinimumWidth(comboBoxwidth*1.5);
  this->levelComboBox->setMinimumHeight(comboBoxHeight);

  QPushButton *addLevelButton = new QPushButton("+");

  levelLayout->addWidget(this->levelComboBox);
  levelLayout->addWidget(addLevelButton);

  connect(this->levelComboBox, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnCurrentLevelChanged(int)));

  connect(addLevelButton, SIGNAL(clicked()), this, SLOT(OnAddLevel()));

  this->setLayout(levelLayout);
}

//////////////////////////////////////////////////
LevelWidget::~LevelWidget()
{
}

//////////////////////////////////////////////////
void LevelWidget::OnCurrentLevelChanged(int _level)
{
  gui::Events::changeLevel(_level);
}

//////////////////////////////////////////////////
void LevelWidget::OnAddLevel()
{
  std::stringstream levelText;
  int count = this->levelComboBox->count();
  levelText << "Level " << (count + 1);
  this->levelComboBox->addItem(QString(levelText.str().c_str()));
  gui::Events::addLevel();
  this->levelComboBox->setCurrentIndex(count);
}
