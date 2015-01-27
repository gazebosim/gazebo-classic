/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/SaveDialog.hh"
#include "gazebo/gui/model/ImportDialog.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelEditorPalette.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelEditorPalette::ModelEditorPalette(QWidget *_parent)
    : QWidget(_parent)
{
  this->setObjectName("modelEditorPalette");

  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Simple Shapes
  QLabel *shapesLabel = new QLabel(tr(
       "<font size=4 color='white'>Simple Shapes</font>"));

  QHBoxLayout *shapesLayout = new QHBoxLayout;

  QSize toolButtonSize(70, 70);
  QSize iconSize(40, 40);

  // Cylinder button
  QToolButton *cylinderButton = new QToolButton(this);
  cylinderButton->setFixedSize(toolButtonSize);
  cylinderButton->setToolTip(tr("Cylinder"));
  cylinderButton->setIcon(QPixmap(":/images/cylinder.png"));
  cylinderButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  cylinderButton->setIconSize(QSize(iconSize));
  cylinderButton->setCheckable(true);
  cylinderButton->setChecked(false);
  connect(cylinderButton, SIGNAL(clicked()), this, SLOT(OnCylinder()));
  shapesLayout->addWidget(cylinderButton);

  // Sphere button
  QToolButton *sphereButton = new QToolButton(this);
  sphereButton->setFixedSize(toolButtonSize);
  sphereButton->setToolTip(tr("Sphere"));
  sphereButton->setIcon(QPixmap(":/images/sphere.png"));
  sphereButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  sphereButton->setIconSize(QSize(iconSize));
  sphereButton->setCheckable(true);
  sphereButton->setChecked(false);
  connect(sphereButton, SIGNAL(clicked()), this, SLOT(OnSphere()));
  shapesLayout->addWidget(sphereButton);

  // Box button
  QToolButton *boxButton = new QToolButton(this);
  boxButton->setFixedSize(toolButtonSize);
  boxButton->setToolTip(tr("Box"));
  boxButton->setIcon(QPixmap(":/images/box.png"));
  boxButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  boxButton->setIconSize(QSize(iconSize));
  boxButton->setCheckable(true);
  boxButton->setChecked(false);
  connect(boxButton, SIGNAL(clicked()), this, SLOT(OnBox()));
  shapesLayout->addWidget(boxButton);

  // Custom Shapes
  QLabel *customShapesLabel = new QLabel(tr(
       "<font size=4 color='white'>Custom Shapes</font>"));

  QHBoxLayout *customLayout = new QHBoxLayout;
  customLayout->setAlignment(Qt::AlignLeft);
  customLayout->addItem(new QSpacerItem(30, 30, QSizePolicy::Minimum,
      QSizePolicy::Minimum));

  QPushButton *customButton = new QPushButton(tr("Add"), this);
  customButton->setMaximumWidth(60);
  customButton->setCheckable(true);
  customButton->setChecked(false);
  connect(customButton, SIGNAL(clicked()), this, SLOT(OnCustom()));
  customLayout->addWidget(customButton, 0, 0);

  // Button group
  this->partButtonGroup = new QButtonGroup;
  this->partButtonGroup->addButton(cylinderButton);
  this->partButtonGroup->addButton(sphereButton);
  this->partButtonGroup->addButton(boxButton);
  this->partButtonGroup->addButton(customButton);

  // Model Settings
  QLabel *settingsLabel = new QLabel(tr(
       "<font size=4 color='white'>Model Settings</font>"));

  QGridLayout *dynamicsLayout = new QGridLayout;

  QLabel *staticLabel = new QLabel(tr("Static:"));
  this->staticCheck = new QCheckBox;
  this->staticCheck->setChecked(false);
  connect(this->staticCheck, SIGNAL(clicked()), this, SLOT(OnStatic()));

  QLabel *autoDisableLabel = new QLabel(tr("Auto-disable:"));
  this->autoDisableCheck = new QCheckBox;
  this->autoDisableCheck->setChecked(true);
  connect(this->autoDisableCheck, SIGNAL(clicked()), this,
      SLOT(OnAutoDisable()));

  dynamicsLayout->addWidget(staticLabel, 0, 0);
  dynamicsLayout->addWidget(this->staticCheck, 0, 1);
  dynamicsLayout->addWidget(autoDisableLabel, 1, 0);
  dynamicsLayout->addWidget(this->autoDisableCheck, 1, 1);

  // save buttons
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

  this->modelCreator = new ModelCreator();
  connect(modelCreator, SIGNAL(PartAdded()), this, SLOT(OnPartAdded()));

  // Horizontal separator
  QFrame *separator = new QFrame(this);
  separator->setFrameShape(QFrame::HLine);
  separator->setFrameShadow(QFrame::Sunken);
  separator->setLineWidth(1);

  // Main layout
  mainLayout->addWidget(shapesLabel);
  mainLayout->addLayout(shapesLayout);
  mainLayout->addWidget(customShapesLabel);
  mainLayout->addLayout(customLayout);
  mainLayout->addItem(new QSpacerItem(30, 30, QSizePolicy::Minimum,
      QSizePolicy::Minimum));
  mainLayout->addWidget(separator);
  mainLayout->addWidget(settingsLabel);
  mainLayout->addLayout(dynamicsLayout);
  mainLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
  mainLayout->addItem(new QSpacerItem(30, 30, QSizePolicy::Minimum,
      QSizePolicy::Minimum));
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->saved = false;
  this->saveLocation = QDir::homePath().toStdString();
  this->modelName = "default";

  KeyEventHandler::Instance()->AddPressFilter("model_editor",
    boost::bind(&ModelEditorPalette::OnKeyPress, this, _1));
}

/////////////////////////////////////////////////
ModelEditorPalette::~ModelEditorPalette()
{
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnItemSelection(QTreeWidgetItem *_item,
                                         int /*_column*/)
{
  if (_item && _item->childCount() > 0)
    _item->setExpanded(!_item->isExpanded());
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnCylinder()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->modelCreator->AddPart(ModelCreator::PART_CYLINDER);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSphere()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->modelCreator->AddPart(ModelCreator::PART_SPHERE);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnBox()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->modelCreator->AddPart(ModelCreator::PART_BOX);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnCustom()
{
  ImportDialog importDialog(this);
  importDialog.deleteLater();
  if (importDialog.exec() == QDialog::Accepted)
  {
    event::Events::setSelectedEntity("", "normal");
    g_arrowAct->trigger();
    this->modelCreator->AddCustom(importDialog.GetImportPath());
  }
  else
  {
    // this unchecks the custom button
    this->OnPartAdded();
  }
}

/////////////////////////////////////////////////
void ModelEditorPalette::AddJoint(const std::string &_type)
{
  event::Events::setSelectedEntity("", "normal");
  this->modelCreator->AddJoint(_type);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnPartAdded()
{
  this->partButtonGroup->setExclusive(false);
  if (this->partButtonGroup->checkedButton())
    this->partButtonGroup->checkedButton()->setChecked(false);
  this->partButtonGroup->setExclusive(true);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnAutoDisable()
{
  this->modelCreator->SetAutoDisable(this->autoDisableCheck->isChecked());
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnStatic()
{
  this->modelCreator->SetStatic(this->staticCheck->isChecked());
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSave()
{
  SaveDialog saveDialog;
  saveDialog.deleteLater();
  saveDialog.SetTitle("Save Model");
  saveDialog.SetSaveName(this->modelCreator->GetModelName());
  saveDialog.SetSaveLocation(QDir::homePath().toStdString());
  saveDialog.SetFileExtension("sdf");
  if (saveDialog.exec() == QDialog::Accepted)
  {
    this->modelName = saveDialog.GetSaveName();
    this->saveLocation = saveDialog.GetSaveLocation();
    this->modelCreator->SetModelName(this->modelName);
    this->modelCreator->GenerateSDF();
    this->modelCreator->SaveToSDF(this->saveLocation);
    this->saveButton->setText("&Save");
  }
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnDiscard()
{
  int ret = QMessageBox::warning(0, QString("Discard"),
      QString("Are you sure you want to discard\n"
      "your model? All of your work will\n"
      "be lost."),
      QMessageBox::Discard | QMessageBox::Cancel,
      QMessageBox::Cancel);

  switch (ret)
  {
    case QMessageBox::Discard:
      this->modelCreator->Reset();
      this->saveButton->setText("&Save As");
      this->saveLocation = QDir::homePath().toStdString();
      break;
    case QMessageBox::Cancel:
      // Do nothing
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnDone()
{
  SaveDialog saveDialog;
  saveDialog.SetTitle("Save Model");
  saveDialog.SetSaveName(this->modelCreator->GetModelName());
  saveDialog.SetSaveLocation(QDir::homePath().toStdString());
  saveDialog.SetFileExtension("sdf");
  if (saveDialog.exec() == QDialog::Accepted)
  {
    this->modelName = saveDialog.GetSaveName();
    this->saveLocation = saveDialog.GetSaveLocation();
    this->modelCreator->SetModelName(this->modelName);
    this->modelCreator->GenerateSDF();
    this->modelCreator->SaveToSDF(this->saveLocation);
    this->modelCreator->FinishModel();
    gui::model::Events::finishModel();
  }
}

/////////////////////////////////////////////////
bool ModelEditorPalette::OnKeyPress(const common::KeyEvent &_event)
{
  if (_event.key == Qt::Key_Escape)
  {
    // call the slots to uncheck the buttons
    this->OnPartAdded();
  }
  if (_event.key == Qt::Key_Delete)
  {
    event::Events::setSelectedEntity("", "normal");
    g_arrowAct->trigger();
  }
  return false;
}

/////////////////////////////////////////////////
ModelCreator *ModelEditorPalette::GetModelCreator()
{
  return this->modelCreator;
}
