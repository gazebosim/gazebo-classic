/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/model/ModelEditorPalette.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
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

  connect(g_editModelAct, SIGNAL(triggered(bool)), this, SLOT(OnEdit(bool)));

  this->connections.push_back(
      gui::model::Events::ConnectFinishModel(
      boost::bind(&ModelEditor::OnFinish, this)));

  // Add a joint icon to the render widget toolbar
  QToolBar *toolbar = this->mainWindow->GetRenderWidget()->GetToolbar();
  QToolButton *jointButton = new QToolButton(toolbar);
  jointButton->setIcon(QIcon(":/images/box.png"));
  jointButton->setText(tr("Joint"));
  QMenu *jointMenu = new QMenu(jointButton);
  jointButton->setMenu(jointMenu);
  jointButton->setPopupMode(QToolButton::InstantPopup);
  QAction *revoluteJointAct = new QAction(tr("Revolute"), this);
  QAction *revolute2JointAct = new QAction(tr("Revolute2"), this);
  QAction *prismaticJointAct = new QAction(tr("Prismatic"), this);
  QAction *ballJointAct = new QAction(tr("Ball"), this);
  QAction *universalJointAct = new QAction(tr("Universal"), this);
  QAction *screwJointAct = new QAction(tr("Screw"), this);

  jointMenu->addAction(revoluteJointAct);
  jointMenu->addAction(revolute2JointAct);
  jointMenu->addAction(prismaticJointAct);
  jointMenu->addAction(ballJointAct);
  jointMenu->addAction(universalJointAct);
  jointMenu->addAction(screwJointAct);
  this->jointAct = toolbar->addWidget(jointButton);
  this->jointAct->setVisible(false);


  this->signalMapper = new QSignalMapper(this);
  connect(this->signalMapper, SIGNAL(mapped(const QString)),
      this->modelPalette, SLOT(OnAddJoint(const QString)));

  connect(revoluteJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(revoluteJointAct,
      revoluteJointAct->text());
  connect(revolute2JointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(revolute2JointAct,
      revolute2JointAct->text());
  connect(prismaticJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(prismaticJointAct,
      prismaticJointAct->text());
  connect(ballJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(ballJointAct,
      ballJointAct->text());
  connect(universalJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(universalJointAct,
      universalJointAct->text());
  connect(screwJointAct, SIGNAL(triggered()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(screwJointAct,
      screwJointAct->text());
}

/////////////////////////////////////////////////
ModelEditor::~ModelEditor()
{
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
  event::Events::setSelectedEntity("", "normal");
  this->active = !this->active;
  this->ToggleToolbar();
  g_editModelAct->setChecked(this->active);
}

/////////////////////////////////////////////////
void ModelEditor::OnFinish()
{
  this->OnEdit(g_editModelAct->isChecked());
}

/////////////////////////////////////////////////
void ModelEditor::ToggleToolbar()
{
  QToolBar *toolbar = this->mainWindow->GetRenderWidget()->GetToolbar();
  QList<QAction *> actions = toolbar->actions();

  for (int i = 0; i < actions.size(); ++i)
  {
    actions[i]->setVisible(!this->active);
  }

  if (this->active)
  {
    g_arrowAct->setVisible(true);
    g_rotateAct->setVisible(true);
    g_translateAct->setVisible(true);
    g_scaleAct->setVisible(true);
  }

  this->jointAct->setVisible(this->active);
}
