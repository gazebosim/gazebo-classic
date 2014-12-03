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

  connect(g_editModelAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));

  this->connections.push_back(
      gui::model::Events::ConnectFinishModel(
      boost::bind(&ModelEditor::OnFinish, this)));
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
  this->active = !this->active;
  this->ToggleToolbar();
//  g_editModelAct->setChecked(this->active);
}

/////////////////////////////////////////////////
void ModelEditor::OnFinish()
{
//  this->OnEdit(g_editModelAct->isChecked());
  g_editModelAct->trigger();
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
//        actions[i] == g_copyAct -- issue #1314
//        actions[i] == g_pasteAct
//        align tool              -- issue #1323
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
}
