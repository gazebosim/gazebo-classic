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

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/LevelWidget.hh"
#include "gazebo/gui/building/LevelWidgetPrivate.hh"

using namespace gazebo;
using namespace gui;

//////////////////////////////////////////////////
LevelWidget::LevelWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new LevelWidgetPrivate)
{
  this->setObjectName("levelWidget");
  this->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

  QHBoxLayout *levelLayout = new QHBoxLayout;
  this->dataPtr->levelCounter = 0;

  this->dataPtr->showFloorplanAct = new QAction("Floorplan", this);
  this->dataPtr->showElementsAct = new QAction("Features", this);
  this->dataPtr->showFloorplanAct->setCheckable(true);
  this->dataPtr->showElementsAct->setCheckable(true);
  this->dataPtr->showFloorplanAct->setChecked(true);
  this->dataPtr->showElementsAct->setChecked(true);
  this->dataPtr->showFloorplanAct->setShortcut(tr("F"));
  this->dataPtr->showElementsAct->setShortcut(tr("G"));
  connect(this->dataPtr->showFloorplanAct, SIGNAL(triggered()), this, SLOT(
      OnShowFloorplan()));
  connect(this->dataPtr->showElementsAct, SIGNAL(triggered()), this, SLOT(
      OnShowElements()));

  QMenu *showMenu = new QMenu(this);
  showMenu->addAction(this->dataPtr->showFloorplanAct);
  showMenu->addAction(this->dataPtr->showElementsAct);
  QPushButton *showButton = new QPushButton("View", this);
  showButton->setMenu(showMenu);

  this->dataPtr->levelComboBox = new QComboBox;
  this->dataPtr->levelComboBox->addItem(QString("Level 1"));
  int comboBoxwidth = this->dataPtr->levelComboBox->minimumSizeHint().width();
  int comboBoxHeight = this->dataPtr->levelComboBox->minimumSizeHint().height();
  this->dataPtr->levelComboBox->setMinimumWidth(comboBoxwidth*3);
  this->dataPtr->levelComboBox->setMinimumHeight(comboBoxHeight);
  this->setMinimumWidth(comboBoxwidth*6);

  QPushButton *deleteLevelButton = new QPushButton("-");
  deleteLevelButton->setToolTip("Delete this level");
  QPushButton *addLevelButton = new QPushButton("+");
  addLevelButton->setToolTip("Add new level");

  levelLayout->addWidget(showButton);
  levelLayout->addWidget(this->dataPtr->levelComboBox);
  levelLayout->addWidget(deleteLevelButton);
  levelLayout->addWidget(addLevelButton);

  connect(this->dataPtr->levelComboBox, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnCurrentLevelChanged(int)));
  connect(deleteLevelButton, SIGNAL(clicked()), this, SLOT(OnDeleteLevel()));
  connect(addLevelButton, SIGNAL(clicked()), this, SLOT(OnAddLevel()));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectUpdateLevelWidget(
      std::bind(&LevelWidget::OnUpdateLevelWidget, this, std::placeholders::_1,
      std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectTriggerShowFloorplan(
      std::bind(&LevelWidget::OnTriggerShowFloorplan, this)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectTriggerShowElements(
      std::bind(&LevelWidget::OnTriggerShowElements, this)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectNewBuildingModel(
      std::bind(&LevelWidget::OnDiscard, this)));

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
    this->dataPtr->levelComboBox->removeItem(_level);
    if (_level-1 >= 0)
      this->dataPtr->levelComboBox->setCurrentIndex(_level-1);
    return;
  }

  // Add
  if (_level == this->dataPtr->levelComboBox->count())
  {
    this->dataPtr->levelComboBox->addItem(tr(_newName.c_str()));
    this->dataPtr->levelComboBox->setCurrentIndex(_level);
    this->dataPtr->levelCounter++;
  }
  // Change name
  else
  {
    this->dataPtr->levelComboBox->setItemText(_level, tr(_newName.c_str()));
  }
}

//////////////////////////////////////////////////
void LevelWidget::OnDiscard()
{
  this->dataPtr->levelComboBox->clear();
  this->dataPtr->levelComboBox->addItem(QString("Level 1"));
  this->dataPtr->levelCounter = 0;
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
  this->dataPtr->showFloorplanAct->setChecked(
      !this->dataPtr->showFloorplanAct->isChecked());
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
  this->dataPtr->showElementsAct->setChecked(
      !this->dataPtr->showElementsAct->isChecked());
}
