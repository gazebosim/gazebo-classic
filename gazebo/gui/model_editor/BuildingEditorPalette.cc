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

#include "gazebo/gui/model_editor/BuildingEditorPalette.hh"
#include "gazebo/gui/model_editor/EditorEvents.hh"

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

  QGridLayout *floorPlanGridLayout = new QGridLayout;

  QLabel *floorPlanLabel = new QLabel(tr("Floor Plan"));
  floorPlanLabel->setFont(underLineFont);

  QLabel *drawWallsLabel = new QLabel;
  drawWallsLabel->setText("Draw Walls");
  QPushButton *addDrawWallButton = new QPushButton;
  addDrawWallButton->setIcon(QIcon(":/images/wall.png"));
  addDrawWallButton->setIconSize(QSize(30, 60));
  addDrawWallButton->setFlat(true);
  connect(addDrawWallButton, SIGNAL(clicked()), this, SLOT(OnDrawWall()));

  QVBoxLayout *drawWallLayout = new QVBoxLayout;
  drawWallLayout->addWidget(addDrawWallButton);
  drawWallLayout->addWidget(drawWallsLabel);
  floorPlanGridLayout->addLayout(drawWallLayout, 0, 0);
  floorPlanGridLayout->setAlignment(Qt::AlignLeft);

/*  QLabel *importImageLabel = new QLabel;
  importImageLabel->setText("Import Image");
  QPushButton *importImageButton = new QPushButton;
  importImageButton->setIcon(QIcon(":/images/box.png"));
  importImageButton->setIconSize(QSize(30,30));
  importImageButton->setFlat(true);
  connect(importImageButton, SIGNAL(clicked()), this, SLOT(OnImportImage()));

  QVBoxLayout *importImageLayout = new QVBoxLayout;
  importImageLayout->addWidget(importImageButton);
  importImageLayout->addWidget(importImageLabel);
  importImageLayout->setAlignment(Qt::AlignLeft);
  floorPlanGridLayout->addLayout(importImageLayout, 0, 1);*/

  QVBoxLayout *floorPlanLayout = new QVBoxLayout;
  floorPlanLayout->addWidget(floorPlanLabel);
  floorPlanLayout->addLayout(floorPlanGridLayout);

  QGridLayout *windowDoorGridLayout = new QGridLayout;
  QLabel *windowDoorLabel = new QLabel(tr("Windows & Doors"));
  windowDoorLabel->setFont(underLineFont);

  QLabel *addWindowLabel = new QLabel;
  addWindowLabel->setText("Add Window");
  QPushButton *addWindowButton = new QPushButton;
  addWindowButton->setIcon(QIcon(":/images/window.png"));
  addWindowButton->setIconSize(QSize(30, 60));
  addWindowButton->setFlat(true);
  connect(addWindowButton, SIGNAL(clicked()), this, SLOT(OnAddWindow()));

  QVBoxLayout *addWindowLayout = new QVBoxLayout;
  addWindowLayout->addWidget(addWindowButton);
  addWindowLayout->addWidget(addWindowLabel);
  addWindowLayout->setAlignment(Qt::AlignLeft);
  windowDoorGridLayout->addLayout(addWindowLayout, 0, 0);

  windowDoorGridLayout->addItem(new QSpacerItem(20, 10, QSizePolicy::Minimum,
      QSizePolicy::Minimum), 0, 1);

  QLabel *addDoorLabel = new QLabel;
  addDoorLabel->setText("Add Doorway");
  QPushButton *addDoorButton = new QPushButton;
  addDoorButton->setIcon(QIcon(":/images/door.png"));
  addDoorButton->setIconSize(QSize(30, 60));
  addDoorButton->setFlat(true);
  connect(addDoorButton, SIGNAL(clicked()), this, SLOT(OnAddDoor()));

  QVBoxLayout *addDoorLayout = new QVBoxLayout;
  addDoorLayout->addWidget(addDoorButton);
  addDoorLayout->addWidget(addDoorLabel);
  addDoorLayout->setAlignment(Qt::AlignLeft);
  windowDoorGridLayout->addLayout(addDoorLayout, 0, 2);
  windowDoorGridLayout->setAlignment(Qt::AlignLeft);

  QVBoxLayout *windowDoorLayout = new QVBoxLayout;
  windowDoorLayout->addWidget(windowDoorLabel);
  windowDoorLayout->addLayout(windowDoorGridLayout);

  QLabel *otherLabel = new QLabel(tr("Other"));
  otherLabel->setFont(underLineFont);

  QGridLayout *otherGridLayout = new QGridLayout;

  QLabel *addStairsLabel = new QLabel;
  addStairsLabel->setText("Add Stairs");
  QPushButton *addStairsButton = new QPushButton;
  addStairsButton->setIcon(QIcon(":/images/stairs.png"));
  addStairsButton->setIconSize(QSize(30, 60));
  addStairsButton->setFlat(true);
  connect(addStairsButton, SIGNAL(clicked()), this, SLOT(OnAddStairs()));

  QVBoxLayout *addStairsLayout = new QVBoxLayout;
  addStairsLayout->addWidget(addStairsButton);
  addStairsLayout->addWidget(addStairsLabel);
  otherGridLayout->addLayout(addStairsLayout, 0, 0);
  otherGridLayout->setAlignment(Qt::AlignLeft);

  QVBoxLayout *otherLayout = new QVBoxLayout;
  otherLayout->addWidget(otherLabel);
  otherLayout->addLayout(otherGridLayout);

  QPushButton *discardButton = new QPushButton(tr("Discard"));
  connect(discardButton, SIGNAL(clicked()), this, SLOT(OnDiscard()));
  saveButton = new QPushButton(tr("Save As"));
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));
  QPushButton *doneButton = new QPushButton(tr("Done"));
  connect(doneButton, SIGNAL(clicked()), this, SLOT(OnDone()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(discardButton);
  buttonsLayout->addWidget(saveButton);
  buttonsLayout->addWidget(doneButton);
  buttonsLayout->setAlignment(Qt::AlignCenter);

  mainLayout->addLayout(modelNameLayout);
  mainLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  mainLayout->addLayout(floorPlanLayout);
  mainLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  mainLayout->addLayout(windowDoorLayout);
  mainLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  mainLayout->addLayout(otherLayout);
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
void BuildingEditorPalette::OnAddStairs()
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
