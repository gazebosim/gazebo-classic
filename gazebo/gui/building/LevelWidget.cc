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

#include <boost/bind.hpp>
#include <sstream>
#include "gazebo/gui/building/LevelWidget.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"

using namespace gazebo;
using namespace gui;

//////////////////////////////////////////////////
LevelWidget::LevelWidget(QWidget *_parent) : QWidget(_parent)
{
  this->setObjectName("levelWidget");
  this->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

  QHBoxLayout *levelLayout = new QHBoxLayout;
  this->levelCounter = 0;

  this->showFloorplanAct = new QAction("Floorplan", this);
  this->showElementsAct = new QAction("Features", this);
  this->showFloorplanAct->setCheckable(true);
  this->showElementsAct->setCheckable(true);
  this->showFloorplanAct->setChecked(true);
  this->showElementsAct->setChecked(true);
  this->showFloorplanAct->setShortcut(tr("F"));
  this->showElementsAct->setShortcut(tr("G"));
  connect(this->showFloorplanAct, SIGNAL(triggered()), this, SLOT(
      OnShowFloorplan()));
  connect(this->showElementsAct, SIGNAL(triggered()), this, SLOT(
      OnShowElements()));

  QMenu *showMenu = new QMenu(this);
  showMenu->addAction(this->showFloorplanAct);
  showMenu->addAction(this->showElementsAct);
  QPushButton *showButton = new QPushButton("View", this);
  showButton->setMenu(showMenu);

  this->levelComboBox = new QComboBox;
  this->levelComboBox->addItem(QString("Level 1"));
  int comboBoxwidth = levelComboBox->minimumSizeHint().width();
  int comboBoxHeight = levelComboBox->minimumSizeHint().height();
  this->levelComboBox->setMinimumWidth(comboBoxwidth*3);
  this->levelComboBox->setMinimumHeight(comboBoxHeight);
  this->setMinimumWidth(comboBoxwidth*6);

  QPushButton *deleteLevelButton = new QPushButton("-");
  deleteLevelButton->setToolTip("Delete this level");
  QPushButton *addLevelButton = new QPushButton("+");
  addLevelButton->setToolTip("Add new level");

  levelLayout->addWidget(showButton);
  levelLayout->addWidget(this->levelComboBox);
  levelLayout->addWidget(deleteLevelButton);
  levelLayout->addWidget(addLevelButton);

  connect(this->levelComboBox, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnCurrentLevelChanged(int)));
  connect(deleteLevelButton, SIGNAL(clicked()), this, SLOT(OnDeleteLevel()));
  connect(addLevelButton, SIGNAL(clicked()), this, SLOT(OnAddLevel()));

  this->connections.push_back(
      gui::editor::Events::ConnectUpdateLevelWidget(
      boost::bind(&LevelWidget::OnUpdateLevelWidget, this, _1, _2)));

  this->connections.push_back(
      gui::editor::Events::ConnectTriggerShowFloorplan(
      boost::bind(&LevelWidget::OnTriggerShowFloorplan, this)));

  this->connections.push_back(
      gui::editor::Events::ConnectTriggerShowElements(
      boost::bind(&LevelWidget::OnTriggerShowElements, this)));

  this->connections.push_back(
      gui::editor::Events::ConnectNewBuildingModel(
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
  gui::editor::Events::addBuildingLevel();
}

//////////////////////////////////////////////////
void LevelWidget::OnDeleteLevel()
{
  gui::editor::Events::deleteBuildingLevel();
}

//////////////////////////////////////////////////
void LevelWidget::OnUpdateLevelWidget(int _level, const std::string &_newName)
{
  // Delete
  if (_newName.empty())
  {
    this->levelComboBox->removeItem(_level);
    if (_level-1 >= 0)
      this->levelComboBox->setCurrentIndex(_level-1);
    return;
  }

  // Add
  if (_level == this->levelComboBox->count())
  {
    this->levelComboBox->addItem(tr(_newName.c_str()));
    this->levelComboBox->setCurrentIndex(_level);
    this->levelCounter++;
  }
  // Change name
  else
  {
    this->levelComboBox->setItemText(_level, tr(_newName.c_str()));
  }
}

//////////////////////////////////////////////////
void LevelWidget::OnDiscard()
{
  this->levelComboBox->clear();
  this->levelComboBox->addItem(QString("Level 1"));
  this->levelCounter = 0;
}

//////////////////////////////////////////////////
void LevelWidget::OnShowFloorplan()
{
  gui::editor::Events::showFloorplan();
}

//////////////////////////////////////////////////
void LevelWidget::OnTriggerShowFloorplan()
{
  this->OnShowFloorplan();
  this->showFloorplanAct->setChecked(!this->showFloorplanAct->isChecked());
}

//////////////////////////////////////////////////
void LevelWidget::OnShowElements()
{
  gui::editor::Events::showElements();
}

//////////////////////////////////////////////////
void LevelWidget::OnTriggerShowElements()
{
  this->OnShowElements();
  this->showElementsAct->setChecked(!this->showElementsAct->isChecked());
}
