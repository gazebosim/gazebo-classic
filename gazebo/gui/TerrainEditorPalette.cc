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

#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/TerrainEditorPalette.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TerrainEditorPalette::TerrainEditorPalette(QWidget *_parent)
    : QWidget(_parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QToolButton *raiseButton = new QToolButton(this);
  raiseButton->setIcon(QPixmap(":/images/wall.png"));
  raiseButton->setStatusTip(tr("Record a log file"));
  raiseButton->setCheckable(true);
  raiseButton->setChecked(false);
  raiseButton->setIconSize(QSize(30, 30));
  connect(raiseButton, SIGNAL(toggled(bool)), this, SLOT(OnRaise(bool)));

  QPushButton *lowerButton = new QPushButton;
  lowerButton->setIcon(QIcon(":/images/wall.png"));
  lowerButton->setIconSize(QSize(30, 60));
  lowerButton->setFlat(true);
  connect(lowerButton, SIGNAL(clicked()), this, SLOT(OnLower()));

  QPushButton *saveButton = new QPushButton;
  saveButton->setIcon(QIcon(":/images/pause.png"));
  saveButton->setIconSize(QSize(30, 60));
  saveButton->setFlat(true);
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));

  this->brushSizeSlider = new QSlider(this);
  this->brushSizeSlider->setRange(0, 100);
  this->brushSizeSlider->setTickInterval(1);
  this->brushSizeSlider->setOrientation(Qt::Horizontal);
  this->brushSizeSlider->setValue(20);

  this->brushWeightSlider = new QSlider(this);
  this->brushWeightSlider->setRange(0, 100);
  this->brushWeightSlider->setTickInterval(1);
  this->brushWeightSlider->setOrientation(Qt::Horizontal);
  this->brushWeightSlider->setValue(10);

  mainLayout->addWidget(raiseButton);
  mainLayout->addWidget(lowerButton);
  mainLayout->addWidget(saveButton);
  mainLayout->addWidget(this->brushSizeSlider);
  mainLayout->addWidget(this->brushWeightSlider);
  mainLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);

  this->setObjectName("terrainEditorPalette");
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
TerrainEditorPalette::~TerrainEditorPalette()
{
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnRaise(bool _toggle)
{
  if (_toggle)
  {
    MouseEventHandler::Instance()->AddPressFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMouseMove, this, _1));
  }
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnLower()
{
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnSave()
{
  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  // Get a pointer to the heightmap, if the scen is valid.
  rendering::Heightmap *heightmap = scene ? scene->GetHeightmap() : NULL;
  common::Image img = heightmap->GetImage();

  std::string filename = QFileDialog::getSaveFileName(this,
      tr("Save Heightmap"), QString(),
      tr("PNG Files (*.png)")).toStdString();

  // Return if the user has canceled.
  if (filename.empty())
    return;

  img.SavePNG(filename);
}

/////////////////////////////////////////////////
bool TerrainEditorPalette::OnMousePress(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
    return false;

  bool handled = false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  // Get a pointer to the heightmap, if the scen is valid.
  rendering::Heightmap *heightmap = scene ? scene->GetHeightmap() : NULL;

  // Only try to modify if the heightmap exists, and the LEFT mouse button
  // was used.
  if (heightmap)
  {
    // Get the brush weight and size from the sliders.
    double brushWeight = this->brushWeightSlider->value() / 100.0;
    double brushSize = this->brushSizeSlider->value() / 100.0;

    // Holding shift will lower the terrian instead of raising it.
    if (_event.shift)
      handled = heightmap->Lower(camera, _event.pos, brushSize, brushWeight);
    else
      handled = heightmap->Raise(camera, _event.pos, brushSize, brushWeight);
  }

  return handled;
}

/////////////////////////////////////////////////
bool TerrainEditorPalette::OnMouseMove(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
    return false;

  bool handled = false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  // Get a pointer to the heightmap, if the scen is valid.
  rendering::Heightmap *heightmap = scene ? scene->GetHeightmap() : NULL;

  if (heightmap && _event.dragging)
  {
    // Get the brush weight and size from the sliders.
    double brushWeight = this->brushWeightSlider->value() / 100.0;
    double brushSize = this->brushSizeSlider->value() / 100.0;

    if (_event.shift)
      handled = heightmap->Lower(camera, _event.pos, brushSize, brushWeight);
    else
      handled = heightmap->Raise(camera, _event.pos, brushSize, brushWeight);
  }

  return handled;
}
