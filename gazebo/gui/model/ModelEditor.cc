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
#include "gazebo/gui/model/ModelEditor.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelEditor::ModelEditor(MainWindow *_mainWindow)
  : Editor(_mainWindow)
{
  this->active = false;
  // Create the model editor tab
  this->modelPalette = new ModelEditorPalette(_mainWindow);
  this->Init("modelEditorTab", "Model Editor", this->modelPalette);

  connect(g_editModelAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));

  this->connections.push_back(
      gui::model::Events::ConnectFinishModel(
      boost::bind(&ModelEditor::OnFinish, this)));

  // Add a joint icon to the render widget toolbar
  this->jointAct  = new QAction(QIcon(":/images/draw_link.svg"),
      tr("Joint"), this);
  this->jointAct->setCheckable(true);

  // set up the action group so that only one action is active at one time.
  QActionGroup *actionGroup = g_arrowAct->actionGroup();
  if (actionGroup)
  {
    this->jointAct->setActionGroup(actionGroup);
    connect(actionGroup, SIGNAL(triggered(QAction *)),
        this, SLOT(OnAction(QAction *)));
  }

  QToolBar *toolbar = this->mainWindow->GetRenderWidget()->GetToolbar();
  this->jointButton = new QToolButton(toolbar);
  this->jointButton->setObjectName("jointToolButton");
  this->jointButton->setCheckable(false);
  this->jointButton->setFixedWidth(15);
  this->jointButton->setPopupMode(QToolButton::InstantPopup);
  QMenu *jointMenu = new QMenu(this->jointButton);
  this->jointButton->setMenu(jointMenu);
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

  this->jointSeparatorAct = toolbar->addSeparator();
  toolbar->addAction(this->jointAct);
  this->jointTypeAct = toolbar->addWidget(this->jointButton);
  this->jointAct->setVisible(false);
  this->jointSeparatorAct->setVisible(false);
  this->jointTypeAct->setVisible(false);

  this->signalMapper = new QSignalMapper(this);
  connect(this->signalMapper, SIGNAL(mapped(const QString)),
      this, SLOT(OnAddJoint(const QString)));

  connect(revoluteJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(revoluteJointAct,
      revoluteJointAct->text().toLower());
  connect(revolute2JointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(revolute2JointAct,
      revolute2JointAct->text().toLower());
  connect(prismaticJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(prismaticJointAct,
      prismaticJointAct->text().toLower());
  connect(ballJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(ballJointAct,
      ballJointAct->text().toLower());
  connect(universalJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(universalJointAct,
      universalJointAct->text().toLower());
  connect(screwJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(screwJointAct,
      screwJointAct->text().toLower());

  // set default joint type.
  revoluteJointAct->setChecked(true);
  this->selectedJointType = revoluteJointAct->text().toLower().toStdString();
  connect(this->jointAct, SIGNAL(triggered()), this,
      SLOT(OnAddSelectedJoint()));

  connect(this->modelPalette->GetModelCreator()->GetJointMaker(),
      SIGNAL(JointAdded()), this, SLOT(OnJointAdded()));
}

/////////////////////////////////////////////////
ModelEditor::~ModelEditor()
{
}

/////////////////////////////////////////////////
void ModelEditor::OnAddSelectedJoint()
{
  this->OnAddJoint(tr(this->selectedJointType.c_str()));
}

/////////////////////////////////////////////////
void ModelEditor::OnAddJoint(const QString &_type)
{
  std::string type = _type.toStdString();
  this->modelPalette->AddJoint(type);
  this->selectedJointType = type;
  this->jointAct->setChecked(true);
  gui::Events::manipMode("joint");
}

/////////////////////////////////////////////////
void ModelEditor::OnJointAdded()
{
  if (this->jointAct->isChecked())
  {
    this->jointAct->setChecked(false);
    g_arrowAct->trigger();
  }
}

/////////////////////////////////////////////////
void ModelEditor::OnEdit(bool /*_checked*/)
{
  if (!this->active)
  {
    this->mainWindow->Pause();
    this->mainWindow->ShowLeftColumnWidget("modelEditorTab");
  }
  else
  {
    this->mainWindow->ShowLeftColumnWidget();
    this->mainWindow->Play();
  }
  this->active = !this->active;
  this->ToggleToolbar();
  // g_editModelAct->setChecked(this->active);
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
  if (_action != this->jointAct)
    this->modelPalette->AddJoint("none");
}

/////////////////////////////////////////////////
void ModelEditor::ToggleToolbar()
{
  QToolBar *toolbar = this->mainWindow->GetRenderWidget()->GetToolbar();
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
      actions[i]->setVisible(!this->active);
    }
  }

  this->jointAct->setVisible(this->active);
  this->jointTypeAct->setVisible(this->active);
  this->jointSeparatorAct->setVisible(this->active);
}
