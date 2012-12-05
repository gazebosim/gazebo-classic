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

#include "BuildingEditorPalette.hh"
#include "FinishModelDialog.hh"
#include "WindowDoorInspectorDialog.hh"
#include "WallInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingEditorPalette::BuildingEditorPalette(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("buildingEditorPalette");

  this->modelName = "building";

  QVBoxLayout *mainLayout = new QVBoxLayout;

  QHBoxLayout *modelNameLayout = new QHBoxLayout;
  QLabel *modelLabel = new QLabel(tr("Model: "));
  modelNameLayout->addWidget(modelLabel);

  modelNameLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));

  QFont underLineFont;
  underLineFont.setUnderline(true);

  QGridLayout *floorPlanLayout = new QGridLayout;
  QLabel *floorPlanLabel = new QLabel(tr("Floor Plan"));
  floorPlanLabel->setFont(underLineFont);
  floorPlanLayout->addWidget(floorPlanLabel, 0,0);
  QLabel *drawWallsLabel = new QLabel;
  drawWallsLabel->setTextFormat(Qt::RichText);
  drawWallsLabel->setText("Draw Walls");
  floorPlanLayout->addWidget(drawWallsLabel, 1, 0);
  QLabel *importImageLabel = new QLabel;
  importImageLabel->setTextFormat(Qt::RichText);
  importImageLabel->setText("Import Image");
  floorPlanLayout->addWidget(importImageLabel, 1, 1);
//    lbl->setTextFormat(Qt::RichText);
//    lbl->setText("<img src=":/myimage.png">Hello!");

  QGridLayout *windowDoorLayout = new QGridLayout;
  QLabel *windowDoorLabel = new QLabel(tr("Windows & Doors"));
  windowDoorLabel->setFont(underLineFont);
  windowDoorLayout->addWidget(windowDoorLabel, 0,0);
  QLabel *addWindowLabel = new QLabel;
  addWindowLabel->setTextFormat(Qt::RichText);
  addWindowLabel->setText("Add Window");
  windowDoorLayout->addWidget(addWindowLabel, 1, 0);
  QLabel *addDoorLabel = new QLabel;
  addDoorLabel->setTextFormat(Qt::RichText);
  addDoorLabel->setText("Add Door");
  windowDoorLayout->addWidget(addDoorLabel, 1, 1);

  QGridLayout *otherLayout = new QGridLayout;
  QLabel *otherLabel = new QLabel(tr("Other"));
  otherLabel->setFont(underLineFont);
  otherLayout->addWidget(otherLabel, 0,0);
  QLabel *addStairsLabel = new QLabel;
  addStairsLabel->setTextFormat(Qt::RichText);
  addStairsLabel->setText("Add Stairs");
  otherLayout->addWidget(addStairsLabel, 1, 0);

  QPushButton *discardButton = new QPushButton(tr("Discard"));
  connect(discardButton, SIGNAL(clicked()), this, SLOT(OnDiscard()));
  QPushButton *saveButton = new QPushButton(tr("Save"));
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));
  QPushButton *finishButton = new QPushButton(tr("Finish"));
  connect(finishButton, SIGNAL(clicked()), this, SLOT(OnFinish()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(discardButton);
  buttonsLayout->addWidget(saveButton);
  buttonsLayout->addWidget(finishButton);

  mainLayout->addLayout(modelNameLayout);
  mainLayout->addLayout(floorPlanLayout);
  mainLayout->addLayout(windowDoorLayout);
  mainLayout->addLayout(otherLayout);
  mainLayout->addLayout(buttonsLayout);
  mainLayout->setAlignment(Qt::AlignTop);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);
}

/////////////////////////////////////////////////
BuildingEditorPalette::~BuildingEditorPalette()
{
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
    /// TODO
    break;
    case QMessageBox::Cancel:
    /// TODO
    break;
    default:
    break;
  }

}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnSave()
{
  bool ok;
  QString text = QInputDialog::getText(this, tr("Save"),
      tr("Please give your model a name:"), QLineEdit::Normal,
      QString(this->modelName.c_str()), &ok);
  if (ok && !text.isEmpty())
  {
    /// TODO
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnFinish()
{
//  FinishModelDialog dialog(this);
//  WindowDoorInspectorDialog dialog(0, this);
  WallInspectorDialog dialog(this);
  dialog.exec();

}
