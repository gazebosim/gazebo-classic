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
#include "gazebo/gui/model_editor/FinishModelDialog.hh"
#include "gazebo/gui/model_editor/EditorEvents.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingEditorPalette::BuildingEditorPalette(QWidget *_parent)
    : QWidget(_parent)
{
  this->setObjectName("buildingEditorPalette");

  this->modelName = "MyNamedModel";
  this->saveLocation = QDir::homePath().toStdString();
  this->saved = false;

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
  addDrawWallButton->setIcon(QIcon(":/images/Wall.png"));
  addDrawWallButton->setIconSize(QSize(30,60));
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
  addWindowButton->setIcon(QIcon(":/images/Window.png"));
  addWindowButton->setIconSize(QSize(30,60));
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
  addDoorLabel->setText("Add Door");
  QPushButton *addDoorButton = new QPushButton;
  addDoorButton->setIcon(QIcon(":/images/Door.png"));
  addDoorButton->setIconSize(QSize(30,60));
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
  addStairsButton->setIcon(QIcon(":/images/Stairs.png"));
  addStairsButton->setIconSize(QSize(30,60));
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
  QPushButton *finishButton = new QPushButton(tr("Finish"));
  connect(finishButton, SIGNAL(clicked()), this, SLOT(OnFinish()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(discardButton);
  buttonsLayout->addWidget(saveButton);
  buttonsLayout->addWidget(finishButton);
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

  this->setLayout(mainLayout);
//  this->layout()->setContentsMargins(0, 0, 0, 0);
}

/////////////////////////////////////////////////
BuildingEditorPalette::~BuildingEditorPalette()
{
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDrawWall()
{
  gui::editor::Events::createEditorItem("Wall");
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnImportImage()
{
//  gui::editor::Events::createEditorItem("Image");
}


/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddWindow()
{
  gui::editor::Events::createEditorItem("Window");
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddDoor()
{
  gui::editor::Events::createEditorItem("Door");
}


/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddStairs()
{
  gui::editor::Events::createEditorItem("Stairs");
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDiscard()
{
  int ret = QMessageBox::warning(this, tr("Discard"),
      tr("Are you sure you want to discard\n"
      "your model? All of your work will\n"
      "be lost."),
      QMessageBox::Discard | QMessageBox::Cancel,
      QMessageBox::Discard);

  switch (ret)
  {
    case QMessageBox::Discard:
      gui::editor::Events::discardModel();
      this->saveButton->setText("&Save As");
      this->modelName = "MyNamedModel";
      this->saveLocation = QDir::homePath().toStdString();
      this->modelNameLabel->setText(tr(this->modelName.c_str()));
      this->saved = false;
      break;
    case QMessageBox::Cancel:
    // Do nothing
    break;
    default:
    break;
  }

}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnSave()
{
  if (this->saved)
  {
    gui::editor::Events::saveModel(this->modelName, this->saveLocation);
  }
  else
  {
    FinishModelDialog dialog(FinishModelDialog::MODEL_SAVE, this);
    dialog.SetModelName(this->modelName);
    dialog.SetSaveLocation(this->saveLocation);
    if (dialog.exec() == QDialog::Accepted)
    {
      this->modelName = dialog.GetModelName();
      this->saveLocation = dialog.GetSaveLocation();
      gui::editor::Events::saveModel(this->modelName, this->saveLocation);
      this->saveButton->setText("Save");
      this->modelNameLabel->setText(tr(this->modelName.c_str()));
      this->saved = true;
    }
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnFinish()
{
  FinishModelDialog dialog(FinishModelDialog::MODEL_FINISH, this);
  dialog.SetModelName(this->modelName);
  dialog.SetSaveLocation(this->saveLocation);
  if (dialog.exec() == QDialog::Accepted)
  {
    this->modelName = dialog.GetModelName();
    this->saveLocation = dialog.GetSaveLocation();
    this->modelNameLabel->setText(tr(this->modelName.c_str()));
    gui::editor::Events::saveModel(this->modelName, this->saveLocation);
    gui::editor::Events::finishModel();
  }
}
