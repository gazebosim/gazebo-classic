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

#include "gazebo/gui/qt.h"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/MainWindow.hh"

#include "gazebo/gui/model/ModelEditorPalette.hh"
#include "gazebo/gui/model/ModelEditor.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelEditor::ModelEditor(MainWindow *_mainWindow)
  : Editor(_mainWindow)
{
  // Create the terrain editor tab
  this->modelPalette = new ModelEditorPalette;
  this->Init("modelEditorTab", "Model Editor", this->modelPalette);

  connect(g_editModelAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));
}

/////////////////////////////////////////////////
ModelEditor::~ModelEditor()
{
}

/////////////////////////////////////////////////
void ModelEditor::OnEdit(bool _checked)
{
  if (_checked)
  {
    this->mainWindow->Pause();
    this->mainWindow->ShowLeftColumnWidget("modelEditorTab");
  }
  else
  {
    this->mainWindow->ShowLeftColumnWidget();
    this->mainWindow->Play();
  }
}
