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

#include "gazebo/gui/building/BuildingEditorPalette.hh"
#include "gazebo/gui/building/EditorEvents.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingEditorPalette::BuildingEditorPalette(QWidget *_parent)
    : QWidget(_parent)
{
  this->setObjectName("buildingEditorPalette");

  this->modelName = "BuildingDefaultName";

  QVBoxLayout *mainLayout = new QVBoxLayout;

  QHBoxLayout *modelNameLayout = new QHBoxLayout;
  QLabel *modelLabel = new QLabel(tr("Model: "));
  this->modelNameLabel = new QLabel(tr(this->modelName.c_str()));
  modelNameLayout->addWidget(modelLabel);
  modelNameLayout->addWidget(modelNameLabel);

  modelNameLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));

  QFont underLineFont;
  underLineFont.setUnderline(true);
  underLineFont.setPointSize(14);

  // Add a wall button
  QPushButton *addWallButton = new QPushButton(tr("Add Wall"), this);
  addWallButton->setCheckable(true);
  addWallButton->setChecked(false);
  this->brushes.push_back(addWallButton);
  connect(addWallButton, SIGNAL(clicked()), this, SLOT(OnDrawWall()));

  // Add a window button
  QPushButton *addWindowButton = new QPushButton(tr("Add Window"), this);
  addWindowButton->setCheckable(true);
  addWindowButton->setChecked(false);
  this->brushes.push_back(addWindowButton);
  connect(addWindowButton, SIGNAL(clicked()), this, SLOT(OnAddWindow()));

  // Add a door button
  QPushButton *addDoorButton = new QPushButton(tr("Add Door"), this);
  addDoorButton->setCheckable(true);
  addDoorButton->setChecked(false);
  this->brushes.push_back(addDoorButton);
  connect(addDoorButton, SIGNAL(clicked()), this, SLOT(OnAddDoor()));

  // Add a stair button
  QPushButton *addStairButton = new QPushButton(tr("Add Stair"), this);
  addStairButton->setCheckable(true);
  addStairButton->setChecked(false);
  this->brushes.push_back(addStairButton);
  connect(addStairButton, SIGNAL(clicked()), this, SLOT(OnAddStair()));

  // Layout to hold the drawing buttons
  QGridLayout *gridLayout = new QGridLayout;
  gridLayout->addWidget(addWallButton, 0, 0);
  gridLayout->addWidget(addWindowButton, 0, 1);
  gridLayout->addWidget(addDoorButton, 1, 0);
  gridLayout->addWidget(addStairButton, 1, 1);

  QPushButton *discardButton = new QPushButton(tr("Discard"));
  connect(discardButton, SIGNAL(clicked()), this, SLOT(OnDiscard()));

  this->saveButton = new QPushButton(tr("Save As"));
  connect(this->saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));

  QPushButton *doneButton = new QPushButton(tr("Done"));
  connect(doneButton, SIGNAL(clicked()), this, SLOT(OnDone()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(discardButton);
  buttonsLayout->addWidget(this->saveButton);
  buttonsLayout->addWidget(doneButton);
  buttonsLayout->setAlignment(Qt::AlignCenter);

  mainLayout->addLayout(modelNameLayout);
  mainLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  mainLayout->addLayout(gridLayout);
  mainLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  mainLayout->addLayout(buttonsLayout);
  mainLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);

  std::stringstream tipsText;
  tipsText << "<font size=3><p><b> Tips: </b></b>"
      << "<p>Draw Walls: Click/release to start a wall."
      << "<br>Click again to start a new, attached wall.</br>"
      << "<br>Double-click to stop drawing.</br></p>"
      << "<p>Add Window/Doorway: Click/release in Palette, "
      << "click/release again in 2D View to place the object.<p>"
      << "<p>Double-click an object to open an Inspector with configuration "
      << "options.</p>"
      << "<p>Note: Currently, windows & doors are simple holes in the wall.</p>"
      << "<p>Note: Because Gazebo only supports simple primitive shapes, "
      << "all floors will be rectangular.</p>";

  QTextEdit *tipsTextEdit = new QTextEdit(this);
  tipsTextEdit->setObjectName("tipsTextEdit");
  tipsTextEdit->setText(tr(tipsText.str().c_str()));
  tipsTextEdit->setContentsMargins(0, 0, 0, 0);
  tipsTextEdit->setReadOnly(true);

  mainLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  mainLayout->addWidget(tipsTextEdit);

  this->setLayout(mainLayout);


  this->connections.push_back(
      gui::editor::Events::ConnectSaveBuildingModel(
      boost::bind(&BuildingEditorPalette::OnSaveModel, this, _1, _2)));

  this->connections.push_back(
      gui::editor::Events::ConnectDiscardBuildingModel(
      boost::bind(&BuildingEditorPalette::OnDiscardModel, this)));

  this->connections.push_back(
      gui::editor::Events::ConnectCreateBuildingEditorItem(
    boost::bind(&BuildingEditorPalette::OnCreateEditorItem, this, _1)));
}

/////////////////////////////////////////////////
BuildingEditorPalette::~BuildingEditorPalette()
{
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDrawWall()
{
  gui::editor::Events::createBuildingEditorItem("wall");
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddWindow()
{
  gui::editor::Events::createBuildingEditorItem("window");
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddDoor()
{
  gui::editor::Events::createBuildingEditorItem("door");
}


/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddStair()
{
  gui::editor::Events::createBuildingEditorItem("stairs");
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDiscard()
{
  gui::editor::Events::discardBuildingEditor();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnSave()
{
  gui::editor::Events::saveBuildingEditor();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDone()
{
  gui::editor::Events::doneBuildingEditor();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDiscardModel()
{
  this->saveButton->setText("&Save As");
  this->modelNameLabel->setText("MyNamedModel");
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnSaveModel(const std::string &_saveName,
    const std::string &/*_saveLocation*/)
{
  this->saveButton->setText("Save");
  this->modelNameLabel->setText(tr(_saveName.c_str()));
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnCreateEditorItem(const std::string &_type)
{
  if (_type.empty())
  {
    // Uncheck all the buttons
    for (std::list<QPushButton *>::iterator iter = this->brushes.begin();
        iter != this->brushes.end(); ++iter)
    {
      (*iter)->setChecked(false);
    }
  }
}
