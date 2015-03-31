/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/model/ModelEditorPalette.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelCreator.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelEditorPrivate.hh"
#include "gazebo/gui/model/ModelEditor.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelEditor::ModelEditor(MainWindow *_mainWindow)
  : Editor(_mainWindow), dataPtr(new ModelEditorPrivate)
{
  this->dataPtr->active = false;
  // Create the model editor tab
  this->dataPtr->modelPalette = new ModelEditorPalette(_mainWindow);
  this->Init("modelEditorTab", "Model Editor", this->dataPtr->modelPalette);

  this->dataPtr->newAct = new QAction(tr("&New"), this->mainWindow);
  this->dataPtr->newAct->setStatusTip(tr("New"));
  this->dataPtr->newAct->setShortcut(tr("Ctrl+N"));
  this->dataPtr->newAct->setCheckable(false);
  connect(this->dataPtr->newAct, SIGNAL(triggered()), this, SLOT(New()));

  this->dataPtr->saveAct = new QAction(tr("&Save"), this->mainWindow);
  this->dataPtr->saveAct->setStatusTip(tr("Save"));
  this->dataPtr->saveAct->setShortcut(tr("Ctrl+S"));
  this->dataPtr->saveAct->setCheckable(false);
  connect(this->dataPtr->saveAct, SIGNAL(triggered()), this, SLOT(Save()));

  this->dataPtr->saveAsAct = new QAction(tr("&Save As"), this->mainWindow);
  this->dataPtr->saveAsAct->setStatusTip(tr("Save As"));
  this->dataPtr->saveAsAct->setShortcut(tr("Ctrl+Shift+S"));
  this->dataPtr->saveAsAct->setCheckable(false);
  connect(this->dataPtr->saveAsAct, SIGNAL(triggered()), this, SLOT(SaveAs()));

  this->dataPtr->exitAct = new QAction(tr("E&xit Model Editor"),
      this->mainWindow);
  this->dataPtr->exitAct->setStatusTip(tr("Exit Model Editor"));
  this->dataPtr->exitAct->setShortcut(tr("Ctrl+X"));
  this->dataPtr->exitAct->setCheckable(false);
  connect(this->dataPtr->exitAct, SIGNAL(triggered()), this, SLOT(Exit()));

  connect(g_editModelAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));

  this->connections.push_back(
      gui::model::Events::ConnectFinishModel(
      boost::bind(&ModelEditor::OnFinish, this)));

  // Add a joint icon to the render widget toolbar
  this->dataPtr->jointAct  = new QAction(QIcon(":/images/draw_link.svg"),
      tr("Joint"), this);
  this->dataPtr->jointAct->setCheckable(true);

  // set up the action group so that only one action is active at one time.
  QActionGroup *actionGroup = g_arrowAct->actionGroup();
  if (actionGroup)
  {
    this->dataPtr->jointAct->setActionGroup(actionGroup);
    connect(actionGroup, SIGNAL(triggered(QAction *)),
        this, SLOT(OnAction(QAction *)));
  }

  QToolBar *toolbar = this->mainWindow->GetRenderWidget()->GetToolbar();
  this->dataPtr->jointButton = new QToolButton(toolbar);
  this->dataPtr->jointButton->setObjectName("jointToolButton");
  this->dataPtr->jointButton->setCheckable(false);
  this->dataPtr->jointButton->setFixedWidth(15);
  this->dataPtr->jointButton->setPopupMode(QToolButton::InstantPopup);
  QMenu *jointMenu = new QMenu(this->dataPtr->jointButton);
  this->dataPtr->jointButton->setMenu(jointMenu);
  QAction *revoluteJointAct = new QAction(tr("Revolute"), this);
  QAction *revolute2JointAct = new QAction(tr("Revolute2"), this);
  QAction *prismaticJointAct = new QAction(tr("Prismatic"), this);
  QAction *ballJointAct = new QAction(tr("Ball"), this);
  QAction *universalJointAct = new QAction(tr("Universal"), this);
  QAction *screwJointAct = new QAction(tr("Screw"), this);

  revoluteJointAct->setCheckable(true);
  revolute2JointAct->setCheckable(true);
  prismaticJointAct->setCheckable(true);
  ballJointAct->setCheckable(true);
  universalJointAct->setCheckable(true);
  screwJointAct->setCheckable(true);

  jointMenu->addAction(revoluteJointAct);
  jointMenu->addAction(revolute2JointAct);
  jointMenu->addAction(prismaticJointAct);
  jointMenu->addAction(ballJointAct);
  jointMenu->addAction(universalJointAct);
  jointMenu->addAction(screwJointAct);

  QActionGroup *jointActionGroup = new QActionGroup(this);
  jointActionGroup->addAction(revoluteJointAct);
  jointActionGroup->addAction(revolute2JointAct);
  jointActionGroup->addAction(prismaticJointAct);
  jointActionGroup->addAction(ballJointAct);
  jointActionGroup->addAction(universalJointAct);
  jointActionGroup->addAction(screwJointAct);
  jointActionGroup->setExclusive(true);

  this->dataPtr->jointSeparatorAct = toolbar->addSeparator();
  toolbar->addAction(this->dataPtr->jointAct);
  this->dataPtr->jointTypeAct = toolbar->addWidget(this->dataPtr->jointButton);
  this->dataPtr->jointAct->setVisible(false);
  this->dataPtr->jointSeparatorAct->setVisible(false);
  this->dataPtr->jointTypeAct->setVisible(false);

  this->dataPtr->signalMapper = new QSignalMapper(this);
  connect(this->dataPtr->signalMapper, SIGNAL(mapped(const QString)),
      this, SLOT(OnAddJoint(const QString)));

  connect(revoluteJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(revoluteJointAct,
      revoluteJointAct->text().toLower());
  connect(revolute2JointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(revolute2JointAct,
      revolute2JointAct->text().toLower());
  connect(prismaticJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(prismaticJointAct,
      prismaticJointAct->text().toLower());
  connect(ballJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(ballJointAct,
      ballJointAct->text().toLower());
  connect(universalJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(universalJointAct,
      universalJointAct->text().toLower());
  connect(screwJointAct, SIGNAL(triggered()), this->dataPtr->signalMapper,
      SLOT(map()));
  this->dataPtr->signalMapper->setMapping(screwJointAct,
      screwJointAct->text().toLower());

  // set default joint type.
  revoluteJointAct->setChecked(true);
  this->dataPtr->selectedJointType =
      revoluteJointAct->text().toLower().toStdString();
  connect(this->dataPtr->jointAct, SIGNAL(triggered()), this,
      SLOT(OnAddSelectedJoint()));

  connect(this->dataPtr->modelPalette->GetModelCreator()->GetJointMaker(),
      SIGNAL(JointAdded()), this, SLOT(OnJointAdded()));

  this->dataPtr->menuBar = NULL;
}

/////////////////////////////////////////////////
ModelEditor::~ModelEditor()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

////////////////////////////////////////////////
void ModelEditor::AddItemToPalette(QWidget *_item,
    const std::string &_category)
{
  if (!_item)
  {
    gzerr << "Item is NULL" << std::endl;
    return;
  }

  this->dataPtr->modelPalette->AddItem(_item, _category);
}

////////////////////////////////////////////////
void ModelEditor::Save()
{
  gui::model::Events::saveModelEditor();
}

////////////////////////////////////////////////
void ModelEditor::SaveAs()
{
  gui::model::Events::saveAsModelEditor();
}

/////////////////////////////////////////////////
void ModelEditor::New()
{
  gui::model::Events::newModelEditor();
}

/////////////////////////////////////////////////
void ModelEditor::Exit()
{
  gui::model::Events::exitModelEditor();
}

/////////////////////////////////////////////////
void ModelEditor::CreateMenus()
{
  if (this->dataPtr->menuBar)
    return;

  this->dataPtr->menuBar = new QMenuBar;
  this->dataPtr->menuBar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  QMenu *fileMenu = this->dataPtr->menuBar->addMenu(tr("&File"));
  fileMenu->addAction(this->dataPtr->newAct);
  fileMenu->addAction(this->dataPtr->saveAct);
  fileMenu->addAction(this->dataPtr->saveAsAct);
  fileMenu->addAction(this->dataPtr->exitAct);
}

/////////////////////////////////////////////////
void ModelEditor::OnAddSelectedJoint()
{
  this->OnAddJoint(tr(this->dataPtr->selectedJointType.c_str()));
}

/////////////////////////////////////////////////
void ModelEditor::OnAddJoint(const QString &_type)
{
  std::string type = _type.toStdString();
  this->dataPtr->modelPalette->CreateJoint(type);
  this->dataPtr->selectedJointType = type;
  this->dataPtr->jointAct->setChecked(true);
  gui::Events::manipMode("joint");
}

/////////////////////////////////////////////////
void ModelEditor::OnJointAdded()
{
  if (this->dataPtr->jointAct->isChecked())
  {
    this->dataPtr->jointAct->setChecked(false);
    g_arrowAct->trigger();
  }
}

/////////////////////////////////////////////////
void ModelEditor::OnEdit(bool /*_checked*/)
{
  if (!this->dataPtr->active)
  {
    this->CreateMenus();
    this->dataPtr->mainWindowPaused = this->mainWindow->IsPaused();
    this->mainWindow->Pause();
    this->mainWindow->ShowLeftColumnWidget("modelEditorTab");
    this->mainWindow->ShowMenuBar(this->dataPtr->menuBar);
    this->mainWindow->GetRenderWidget()->ShowTimePanel(false);
  }
  else
  {
    this->mainWindow->ShowLeftColumnWidget();
    this->mainWindow->ShowMenuBar();
    this->mainWindow->GetRenderWidget()->ShowTimePanel(true);
    if (!this->dataPtr->mainWindowPaused)
      this->mainWindow->Play();
  }
  this->dataPtr->active = !this->dataPtr->active;
  this->ToggleToolbar();
  // g_editModelAct->setChecked(this->dataPtr->active);
}

/////////////////////////////////////////////////
void ModelEditor::OnFinish()
{
//  this->OnEdit(g_editModelAct->isChecked());
  g_editModelAct->trigger();
}

/////////////////////////////////////////////////
void ModelEditor::OnAction(QAction *_action)
{
  if (_action != this->dataPtr->jointAct)
    this->dataPtr->modelPalette->CreateJoint("none");
}

/////////////////////////////////////////////////
void ModelEditor::ToggleToolbar()
{
  QToolBar *toolbar =
      this->mainWindow->GetRenderWidget()->GetToolbar();
  QList<QAction *> actions = toolbar->actions();

  for (int i = 0; i < actions.size(); ++i)
  {
    if (actions[i] == g_arrowAct ||
        actions[i] == g_rotateAct ||
        actions[i] == g_translateAct ||
        actions[i] == g_scaleAct ||
        actions[i] == g_screenshotAct ||
        actions[i] == g_copyAct ||
        actions[i] == g_pasteAct ||
        actions[i] == g_alignButtonAct ||
        actions[i] == g_snapAct)
    {
      actions[i]->setVisible(true);
      if (i > 0 && actions[i-1]->isSeparator())
      {
        actions[i-1]->setVisible(true);
      }
    }
    else
    {
      actions[i]->setVisible(!this->dataPtr->active);
    }
  }

  this->dataPtr->jointAct->setVisible(this->dataPtr->active);
  this->dataPtr->jointTypeAct->setVisible(this->dataPtr->active);
  this->dataPtr->jointSeparatorAct->setVisible(this->dataPtr->active);
}
