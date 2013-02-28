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
#include "gazebo/gui/terrain/TerrainEditorPalette.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TerrainEditorPalette::TerrainEditorPalette(QWidget *_parent)
    : QWidget(_parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QPushButton *raiseButton = new QPushButton("Raise", this);
  raiseButton->setStatusTip(tr("Left-mouse press to raise terrain."));
  raiseButton->setCheckable(true);
  raiseButton->setChecked(false);
  connect(raiseButton, SIGNAL(toggled(bool)), this, SLOT(OnRaise(bool)));

  QPushButton *lowerButton = new QPushButton("Lower", this);
  lowerButton->setStatusTip(tr("Left-mouse press to lower terrain."));
  lowerButton->setCheckable(true);
  lowerButton->setChecked(false);
  connect(lowerButton, SIGNAL(toggled(bool)), this, SLOT(OnLower(bool)));

  QPushButton *smoothButton = new QPushButton("Smooth", this);
  smoothButton->setStatusTip(tr("Left-mouse press to smooth terrain."));
  smoothButton->setCheckable(true);
  smoothButton->setChecked(false);

  QPushButton *courseButton = new QPushButton("Course", this);
  courseButton->setStatusTip(tr("Left-mouse press to course terrain."));
  courseButton->setCheckable(true);
  courseButton->setChecked(false);

  QGridLayout *buttonLayout = new QGridLayout;
  buttonLayout->addWidget(raiseButton, 0, 0);
  buttonLayout->addWidget(lowerButton, 0, 1);
  buttonLayout->addWidget(smoothButton, 1, 0);
  buttonLayout->addWidget(courseButton, 1, 1);

  QPushButton *saveButton = new QPushButton("Save Image", this);
  saveButton->setStatusTip(tr("Save terrain as a PNG."));
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));

  this->brushSizeSlider = new QSlider(this);
  this->brushSizeSlider->setRange(0, 100);
  this->brushSizeSlider->setTickInterval(1);
  this->brushSizeSlider->setOrientation(Qt::Horizontal);
  this->brushSizeSlider->setValue(20);

  QHBoxLayout *brushSizeLayout = new QHBoxLayout;
  brushSizeLayout->addWidget(new QLabel(tr("Brush Size: ")));
  brushSizeLayout->addWidget(this->brushSizeSlider);

  this->brushWeightSlider = new QSlider(this);
  this->brushWeightSlider->setRange(0, 100);
  this->brushWeightSlider->setTickInterval(1);
  this->brushWeightSlider->setOrientation(Qt::Horizontal);
  this->brushWeightSlider->setValue(10);

  QHBoxLayout *brushWeightLayout = new QHBoxLayout;
  brushWeightLayout->addWidget(new QLabel(tr("Brush Weight: ")));
  brushWeightLayout->addWidget(this->brushWeightSlider);

  mainLayout->addLayout(buttonLayout);
  mainLayout->addLayout(brushSizeLayout);
  mainLayout->addLayout(brushWeightLayout);
  mainLayout->addStretch(1);
  mainLayout->addWidget(saveButton);

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
    this->state = "raise";
    MouseEventHandler::Instance()->AddPressFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMouseMove, this, _1));
  }
  else
  {
    MouseEventHandler::Instance()->RemovePressFilter("terrain");
    MouseEventHandler::Instance()->RemoveMoveFilter("terrain");
  }
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnLower(bool _toggle)
{
  if (_toggle)
  {
    this->state = "lower";
    MouseEventHandler::Instance()->AddPressFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMouseMove, this, _1));
  }
  else
  {
    MouseEventHandler::Instance()->RemovePressFilter("terrain");
    MouseEventHandler::Instance()->RemoveMoveFilter("terrain");
  }
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
  if (heightmap && !_event.shift)
    handled = this->Apply(_event);

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

  if (heightmap && _event.dragging && !_event.shift)
    handled = this->Apply(_event);

  return handled;
}

/////////////////////////////////////////////////
bool TerrainEditorPalette::Apply(const common::MouseEvent &_event)
{
  bool handled = false;

  // Get the brush weight and size from the sliders.
  double brushWeight = this->brushWeightSlider->value() / 100.0;
  double brushSize = this->brushSizeSlider->value() / 100.0;

  if (this->state == "lower")
    handled = heightmap->Lower(camera, _event.pos, brushSize, brushWeight);
  else if (this->state == "raise")
    handled = heightmap->Raise(camera, _event.pos, brushSize, brushWeight);
  else if (this->state == "smooth")
    handled = heightmap->Smooth(camera, _event.pos, brushSize, brushWeight);
  else if (this->state == "rough")
    handled = heightmap->Roughen(camera, _event.pos, brushSize, brushWeight);
  
  return handled;
}
