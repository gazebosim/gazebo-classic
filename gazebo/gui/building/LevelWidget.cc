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

#include <sstream>
#include "gazebo/gui/building/LevelWidget.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"

using namespace gazebo;
using namespace gui;

//////////////////////////////////////////////////
LevelWidget::LevelWidget(QWidget *_parent) : QWidget(_parent)
{
  this->setObjectName("levelWidget");

  QHBoxLayout *levelLayout = new QHBoxLayout;
  this->levelCounter = 0;

  this->levelComboBox = new QComboBox;
  this->levelComboBox->addItem(QString("Level 1"));
  int comboBoxwidth = levelComboBox->minimumSizeHint().width();
  int comboBoxHeight = levelComboBox->minimumSizeHint().height();
  this->levelComboBox->setMinimumWidth(comboBoxwidth*3);
  this->levelComboBox->setMinimumHeight(comboBoxHeight);

  QPushButton *addLevelButton = new QPushButton("+");
  this->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

  levelLayout->addWidget(this->levelComboBox);
  levelLayout->addWidget(addLevelButton);

  connect(this->levelComboBox, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnCurrentLevelChanged(int)));
  connect(addLevelButton, SIGNAL(clicked()), this, SLOT(OnAddLevel()));

  this->connections.push_back(
    gui::editor::Events::ConnectDeleteBuildingLevel(
    boost::bind(&LevelWidget::OnDeleteLevel, this, _1)));

  this->connections.push_back(
    gui::editor::Events::ConnectChangeBuildingLevelName(
    boost::bind(&LevelWidget::OnChangeLevelName, this, _1, _2)));

  this->connections.push_back(
    gui::editor::Events::ConnectDiscardBuildingModel(
    boost::bind(&LevelWidget::OnDiscard, this)));

  this->setLayout(levelLayout);
}

//////////////////////////////////////////////////
LevelWidget::~LevelWidget()
{
}

//////////////////////////////////////////////////
void LevelWidget::OnCurrentLevelChanged(int _level)
{
  gui::editor::Events::changeBuildingLevel(_level);
}

//////////////////////////////////////////////////
void LevelWidget::OnAddLevel()
{
  std::stringstream levelText;
//  int count = this->levelComboBox->count();
  levelText << "Level " << (++this->levelCounter + 1);
  this->levelComboBox->addItem(QString(levelText.str().c_str()));
  this->levelComboBox->setCurrentIndex(this->levelComboBox->count()-1);
  gui::editor::Events::addBuildingLevel();
}

//////////////////////////////////////////////////
void LevelWidget::OnChangeLevelName(int _level, const std::string &_newName)
{
  if (_level == this->levelComboBox->count())
  {
    // Used for responding to addLevel events from context menus
    // TODO Use a level manager later for managing all events
    this->levelComboBox->addItem(tr(_newName.c_str()));
    this->levelComboBox->setCurrentIndex(_level);
    this->levelCounter++;
  }
  else
  {
    this->levelComboBox->setItemText(_level, tr(_newName.c_str()));
  }
}

//////////////////////////////////////////////////
void LevelWidget::OnDeleteLevel(int _level)
{
  this->levelComboBox->removeItem(_level);
  if (_level-1 >= 0)
    this->levelComboBox->setCurrentIndex(_level-1);
}

//////////////////////////////////////////////////
void LevelWidget::OnDiscard()
{
  this->levelComboBox->clear();
  this->levelComboBox->addItem(QString("Level 1"));
  this->levelCounter = 0;
}
