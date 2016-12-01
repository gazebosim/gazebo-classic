/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/gui/terrain/TerrainEditor.hh"
#include "gazebo/gui/terrain/TerrainEditorPalette.hh"
#include "gazebo/gui/terrain/TerrainEditorPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TerrainEditor::TerrainEditor(MainWindow *_mainWindow)
  : Editor(_mainWindow),
    dataPtr(new TerrainEditorPrivate())
{
  // Create the terrain editor tab
  this->dataPtr->terrainPalette = new TerrainEditorPalette;
  this->Init("terrainEditorTab", "Terrain Editor",
    this->dataPtr->terrainPalette);

  connect(g_editTerrainAct, SIGNAL(toggled(bool)), this, SLOT(OnEdit(bool)));
}

/////////////////////////////////////////////////
TerrainEditor::~TerrainEditor()
{
}

/////////////////////////////////////////////////
void TerrainEditor::OnEdit(bool _checked)
{
  if (_checked)
  {
    this->mainWindow->Pause();
    this->mainWindow->ShowLeftColumnWidget("terrainEditorTab");
  }
  else
  {
    this->mainWindow->ShowLeftColumnWidget();
    this->mainWindow->Play();
  }
}
