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


#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/SaveDialog.hh"
#include "gazebo/gui/model/ImportDialog.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelEditorPalettePrivate.hh"
#include "gazebo/gui/model/ModelEditorPalette.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelEditorPalette::ModelEditorPalette(QWidget *_parent)
    : QWidget(_parent), dataPtr(new ModelEditorPalettePrivate)
{
  this->setObjectName("modelEditorPalette");

  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->dataPtr->modelTreeWidget = new QTreeWidget();
  this->dataPtr->modelTreeWidget->setColumnCount(1);
  this->dataPtr->modelTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->dataPtr->modelTreeWidget->header()->hide();
  this->dataPtr->modelTreeWidget->setFocusPolicy(Qt::NoFocus);

  this->dataPtr->modelTreeWidget->setSelectionMode(
      QAbstractItemView::NoSelection);
  connect(this->dataPtr->modelTreeWidget,
      SIGNAL(itemClicked(QTreeWidgetItem *, int)),
      this, SLOT(OnItemSelection(QTreeWidgetItem *, int)));

  // Create a top-level tree item for the path
  this->dataPtr->modelSettingsItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("Model Settings")));
  this->dataPtr->modelTreeWidget->addTopLevelItem(
      this->dataPtr->modelSettingsItem);

  QTreeWidgetItem *modelSettingsChildItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0));
  this->dataPtr->modelSettingsItem->addChild(modelSettingsChildItem);

  QWidget *modelSettingsWidget = new QWidget;
  QVBoxLayout *modelSettingsLayout = new QVBoxLayout;
  QGridLayout *dynamicsLayout = new QGridLayout;

  QLabel *staticLabel = new QLabel(tr("Static:"));
  this->dataPtr->staticCheck = new QCheckBox;
  this->dataPtr->staticCheck->setText(tr("False"));
  this->dataPtr->staticCheck->setChecked(false);
  connect(this->dataPtr->staticCheck, SIGNAL(clicked()),
      this, SLOT(OnStatic()));

  QLabel *autoDisableLabel = new QLabel(tr("Auto-disable:"));
  this->dataPtr->autoDisableCheck = new QCheckBox;
  this->dataPtr->autoDisableCheck->setText(tr("True"));
  this->dataPtr->autoDisableCheck->setChecked(true);
  connect(this->dataPtr->autoDisableCheck, SIGNAL(clicked()), this,
      SLOT(OnAutoDisable()));

  dynamicsLayout->addWidget(staticLabel, 0, 0);
  dynamicsLayout->addWidget(this->dataPtr->staticCheck, 0, 1);
  dynamicsLayout->addWidget(autoDisableLabel, 1, 0);
  dynamicsLayout->addWidget(this->dataPtr->autoDisableCheck, 1, 1);

  modelSettingsLayout->addLayout(dynamicsLayout);
  modelSettingsWidget->setLayout(modelSettingsLayout);
  this->dataPtr->modelTreeWidget->setItemWidget(modelSettingsChildItem, 0,
    modelSettingsWidget);
  this->dataPtr->modelSettingsItem->setExpanded(true);
  modelSettingsChildItem->setExpanded(true);

  this->dataPtr->modelItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("Parts and Joints")));
  this->dataPtr->modelTreeWidget->addTopLevelItem(this->dataPtr->modelItem);

  QTreeWidgetItem *modelChildItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0));
  this->dataPtr->modelItem->addChild(modelChildItem);

  // Parts and joints buttons
  QWidget *modelWidget = new QWidget;
  QVBoxLayout *modelLayout = new QVBoxLayout;
  QGridLayout *partsLayout = new QGridLayout;
  QLabel *partsLabel = new QLabel(tr("Parts"));

  // cylinder button
  QPushButton *cylinderButton = new QPushButton(tr("Cylinder"), this);
  cylinderButton->setCheckable(true);
  cylinderButton->setChecked(false);
  connect(cylinderButton, SIGNAL(clicked()), this, SLOT(OnCylinder()));

  // Sphere button
  QPushButton *sphereButton = new QPushButton(tr("Sphere"), this);
  sphereButton->setCheckable(true);
  sphereButton->setChecked(false);
  connect(sphereButton, SIGNAL(clicked()), this, SLOT(OnSphere()));

  // Box button
  QPushButton *boxButton = new QPushButton(tr("Box"), this);
  boxButton->setCheckable(true);
  boxButton->setChecked(false);
  connect(boxButton, SIGNAL(clicked()), this, SLOT(OnBox()));

  // Box button
  QPushButton *customButton = new QPushButton(tr("Custom"), this);
  customButton->setCheckable(true);
  customButton->setChecked(false);
  connect(customButton, SIGNAL(clicked()), this, SLOT(OnCustom()));

  partsLayout->addWidget(partsLabel, 0, 0);
  partsLayout->addWidget(cylinderButton, 1, 0);
  partsLayout->addWidget(sphereButton, 1, 1);
  partsLayout->addWidget(boxButton, 2, 0);
  partsLayout->addWidget(customButton, 2, 1);

  QGridLayout *jointsLayout = new QGridLayout;
  QLabel *jointsLabel = new QLabel(tr("Joints"));

  // Fixed joint button
  QPushButton *fixedJointButton = new QPushButton(tr("Fixed"), this);
  fixedJointButton->setCheckable(true);
  fixedJointButton->setChecked(false);
  connect(fixedJointButton, SIGNAL(clicked()), this, SLOT(OnFixedJoint()));

  // Hinge joint button
  QPushButton *hingeJointButton = new QPushButton(tr("Revolute"), this);
  hingeJointButton->setCheckable(true);
  hingeJointButton->setChecked(false);
  connect(hingeJointButton, SIGNAL(clicked()), this, SLOT(OnHingeJoint()));

  // Hinge2 joint button
  QPushButton *hinge2JointButton = new QPushButton(tr("Revolute2"), this);
  hinge2JointButton->setCheckable(true);
  hinge2JointButton->setChecked(false);
  connect(hinge2JointButton, SIGNAL(clicked()), this, SLOT(OnHinge2Joint()));

  // slider joint button
  QPushButton *sliderJointButton = new QPushButton(tr("Prismatic"), this);
  sliderJointButton->setCheckable(true);
  sliderJointButton->setChecked(false);
  connect(sliderJointButton, SIGNAL(clicked()), this, SLOT(OnSliderJoint()));

  // Screw joint button
  QPushButton *screwJointButton = new QPushButton(tr("Screw"), this);
  screwJointButton->setCheckable(true);
  screwJointButton->setChecked(false);
  connect(screwJointButton, SIGNAL(clicked()), this, SLOT(OnScrewJoint()));

  // Universal joint button
  QPushButton *universalJointButton = new QPushButton(tr("Universal"), this);
  universalJointButton->setCheckable(true);
  universalJointButton->setChecked(false);
  connect(universalJointButton, SIGNAL(clicked()), this,
      SLOT(OnUniversalJoint()));

  // Ball joint button
  QPushButton *ballJointButton = new QPushButton(tr("Ball"), this);
  ballJointButton->setCheckable(true);
  ballJointButton->setChecked(false);
  connect(ballJointButton, SIGNAL(clicked()), this, SLOT(OnBallJoint()));

  this->dataPtr->partJointsButtonGroup = new QButtonGroup;
  this->dataPtr->partJointsButtonGroup->addButton(cylinderButton);
  this->dataPtr->partJointsButtonGroup->addButton(sphereButton);
  this->dataPtr->partJointsButtonGroup->addButton(boxButton);
  this->dataPtr->partJointsButtonGroup->addButton(customButton);
  this->dataPtr->partJointsButtonGroup->addButton(fixedJointButton);
  this->dataPtr->partJointsButtonGroup->addButton(sliderJointButton);
  this->dataPtr->partJointsButtonGroup->addButton(hingeJointButton);
  this->dataPtr->partJointsButtonGroup->addButton(hinge2JointButton);
  this->dataPtr->partJointsButtonGroup->addButton(screwJointButton);
  this->dataPtr->partJointsButtonGroup->addButton(universalJointButton);
  this->dataPtr->partJointsButtonGroup->addButton(ballJointButton);

  jointsLayout->addWidget(jointsLabel, 0, 0);
//  jointsLayout->addWidget(fixedJointButton, 1, 0);
  jointsLayout->addWidget(sliderJointButton, 1, 0);
  jointsLayout->addWidget(hingeJointButton, 1, 1);
  jointsLayout->addWidget(hinge2JointButton, 2, 0);
  jointsLayout->addWidget(screwJointButton, 2, 1);
  jointsLayout->addWidget(universalJointButton, 3, 0);
  jointsLayout->addWidget(ballJointButton, 3, 1);

  modelLayout->addLayout(partsLayout);
  modelLayout->addLayout(jointsLayout);
  modelWidget->setLayout(modelLayout);
  this->dataPtr->modelTreeWidget->setItemWidget(modelChildItem, 0, modelWidget);
  this->dataPtr->modelItem->setExpanded(true);
  modelChildItem->setExpanded(true);

  // plugin
  this->dataPtr->pluginItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("Plugin")));
  // this->dataPtr->modelTreeWidget->addTopLevelItem(this->dataPtr->pluginItem);

  // save buttons
  QPushButton *discardButton = new QPushButton(tr("Discard"));
  connect(discardButton, SIGNAL(clicked()), this, SLOT(OnDiscard()));

  this->dataPtr->saveButton = new QPushButton(tr("Save As"));
  connect(this->dataPtr->saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));

  QPushButton *doneButton = new QPushButton(tr("Done"));
  connect(doneButton, SIGNAL(clicked()), this, SLOT(OnDone()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(discardButton);
  buttonsLayout->addWidget(this->dataPtr->saveButton);
  buttonsLayout->addWidget(doneButton);
  buttonsLayout->setAlignment(Qt::AlignCenter);

  this->dataPtr->modelCreator = new ModelCreator();

  connect(this->dataPtr->modelCreator->GetJointMaker(), SIGNAL(JointAdded()),
      this, SLOT(OnJointAdded()));
  connect(this->dataPtr->modelCreator, SIGNAL(PartAdded()), this,
      SLOT(OnPartAdded()));

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->addWidget(this->dataPtr->modelTreeWidget, 0);
  frameLayout->addLayout(buttonsLayout);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);

  mainLayout->addWidget(frame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->dataPtr->saved = false;
  this->dataPtr->saveLocation = QDir::homePath().toStdString();
  this->dataPtr->modelName = "default";

  KeyEventHandler::Instance()->AddPressFilter("model_editor",
    boost::bind(&ModelEditorPalette::OnKeyPress, this, _1));
}

/////////////////////////////////////////////////
ModelEditorPalette::~ModelEditorPalette()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
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
  this->dataPtr->modelCreator->AddPart(ModelCreator::PART_CYLINDER);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSphere()
{
  this->dataPtr->modelCreator->AddPart(ModelCreator::PART_SPHERE);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnBox()
{
  this->dataPtr->modelCreator->AddPart(ModelCreator::PART_BOX);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnCustom()
{
  ImportDialog importDialog;
  importDialog.deleteLater();
  if (importDialog.exec() == QDialog::Accepted)
  {
    this->dataPtr->modelCreator->AddCustom(importDialog.GetImportPath());
  }
  else
  {
    // this unchecks the custom button
    this->OnPartAdded();
  }
}


/////////////////////////////////////////////////
void ModelEditorPalette::OnFixedJoint()
{
  this->dataPtr->modelCreator->AddJoint(JointMaker::JOINT_FIXED);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnHingeJoint()
{
  this->dataPtr->modelCreator->AddJoint(JointMaker::JOINT_HINGE);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnHinge2Joint()
{
  this->dataPtr->modelCreator->AddJoint(JointMaker::JOINT_HINGE2);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSliderJoint()
{
  this->dataPtr->modelCreator->AddJoint(JointMaker::JOINT_SLIDER);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnScrewJoint()
{
  this->dataPtr->modelCreator->AddJoint(JointMaker::JOINT_SCREW);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnUniversalJoint()
{
  this->dataPtr->modelCreator->AddJoint(JointMaker::JOINT_UNIVERSAL);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnBallJoint()
{
  this->dataPtr->modelCreator->AddJoint(JointMaker::JOINT_BALL);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnJointAdded()
{
  this->dataPtr->partJointsButtonGroup->setExclusive(false);
  if (this->dataPtr->partJointsButtonGroup->checkedButton())
    this->dataPtr->partJointsButtonGroup->checkedButton()->setChecked(false);
  this->dataPtr->partJointsButtonGroup->setExclusive(true);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnPartAdded()
{
  this->dataPtr->partJointsButtonGroup->setExclusive(false);
  if (this->dataPtr->partJointsButtonGroup->checkedButton())
    this->dataPtr->partJointsButtonGroup->checkedButton()->setChecked(false);
  this->dataPtr->partJointsButtonGroup->setExclusive(true);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnAutoDisable()
{
  std::string text = this->dataPtr->autoDisableCheck->isChecked() ? "True" : "False";
  this->dataPtr->autoDisableCheck->setText(tr(text.c_str()));
  this->dataPtr->modelCreator->SetAutoDisable(this->dataPtr->autoDisableCheck->isChecked());
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnStatic()
{
  std::string text = this->dataPtr->staticCheck->isChecked() ? "True" : "False";
  this->dataPtr->staticCheck->setText(tr(text.c_str()));
  this->dataPtr->modelCreator->SetStatic(this->dataPtr->staticCheck->isChecked());
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSave()
{
  SaveDialog saveDialog;
  saveDialog.deleteLater();
  saveDialog.SetTitle("Save Model");
  saveDialog.SetSaveName(this->dataPtr->modelCreator->GetModelName());
  saveDialog.SetSaveLocation(QDir::homePath().toStdString());
  saveDialog.SetFileExtension("sdf");
  if (saveDialog.exec() == QDialog::Accepted)
  {
    this->dataPtr->modelName = saveDialog.GetSaveName();
    this->dataPtr->saveLocation = saveDialog.GetSaveLocation();
    this->dataPtr->modelCreator->SetModelName(this->dataPtr->modelName);
    this->dataPtr->modelCreator->GenerateSDF();
    this->dataPtr->modelCreator->SaveToSDF(this->dataPtr->saveLocation);
    this->dataPtr->saveButton->setText("&Save");
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
      this->dataPtr->modelCreator->Reset();
      this->dataPtr->saveButton->setText("&Save As");
      this->dataPtr->saveLocation = QDir::homePath().toStdString();
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
  saveDialog.SetSaveName(this->dataPtr->modelCreator->GetModelName());
  saveDialog.SetSaveLocation(QDir::homePath().toStdString());
  saveDialog.SetFileExtension("sdf");
  if (saveDialog.exec() == QDialog::Accepted)
  {
    this->dataPtr->modelName = saveDialog.GetSaveName();
    this->dataPtr->saveLocation = saveDialog.GetSaveLocation();
    this->dataPtr->modelCreator->SetModelName(this->dataPtr->modelName);
    this->dataPtr->modelCreator->GenerateSDF();
    this->dataPtr->modelCreator->SaveToSDF(this->dataPtr->saveLocation);
    this->dataPtr->modelCreator->FinishModel();
    gui::model::Events::finishModel();
  }
}

/////////////////////////////////////////////////
bool ModelEditorPalette::OnKeyPress(const common::KeyEvent &_event)
{
  if (_event.key == Qt::Key_Escape)
  {
    // this->dataPtr->modelCreator->Stop();
    // call the slots to uncheck the buttons
    this->OnPartAdded();
    this->OnJointAdded();
    return true;
  }
  return false;
}
