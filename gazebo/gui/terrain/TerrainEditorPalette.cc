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

  // Create the button to raise terrain
  this->raiseButton = new QPushButton("Raise", this);
  this->raiseButton->setStatusTip(tr("Left-mouse press to raise terrain."));
  this->raiseButton->setCheckable(true);
  this->raiseButton->setChecked(false);
  connect(this->raiseButton, SIGNAL(toggled(bool)), this, SLOT(OnRaise(bool)));

  // Create the button to lower terrain
  this->lowerButton = new QPushButton("Lower", this);
  this->lowerButton->setStatusTip(tr("Left-mouse press to lower terrain."));
  this->lowerButton->setCheckable(true);
  this->lowerButton->setChecked(false);
  connect(this->lowerButton, SIGNAL(toggled(bool)), this, SLOT(OnLower(bool)));

  // Create the button to flatten terrain
  this->flattenButton = new QPushButton("Flatten", this);
  this->flattenButton->setStatusTip(tr("Left-mouse press to flatten terrain."));
  this->flattenButton->setCheckable(true);
  this->flattenButton->setChecked(false);
  connect(this->flattenButton, SIGNAL(toggled(bool)),
          this, SLOT(OnFlatten(bool)));

  // Create the button to roughen terrain
  this->courseButton = new QPushButton("Course", this);
  this->courseButton->setStatusTip(tr("Left-mouse press to course terrain."));
  this->courseButton->setCheckable(true);
  this->courseButton->setChecked(false);
  connect(this->courseButton, SIGNAL(toggled(bool)),
          this, SLOT(OnCourse(bool)));

  // Create the layout to hold all the buttons
  QGridLayout *buttonLayout = new QGridLayout;
  buttonLayout->addWidget(raiseButton, 0, 0);
  buttonLayout->addWidget(lowerButton, 0, 1);
  buttonLayout->addWidget(flattenButton, 1, 0);
  buttonLayout->addWidget(courseButton, 1, 1);

  // Add a save button
  QPushButton *saveButton = new QPushButton("Save Image", this);
  saveButton->setStatusTip(tr("Save terrain as a PNG."));
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));

  // Create a slider to control the size of the brush
  this->brushSizeSlider = new QSlider(this);
  this->brushSizeSlider->setRange(1, 100);
  this->brushSizeSlider->setTickInterval(1);
  this->brushSizeSlider->setOrientation(Qt::Horizontal);
  this->brushSizeSlider->setValue(20);

  // Create a layout to hold the brush size slider and its label
  QHBoxLayout *brushSizeLayout = new QHBoxLayout;
  brushSizeLayout->addWidget(new QLabel(tr("Brush Size: ")));
  brushSizeLayout->addWidget(this->brushSizeSlider);

  // Create a slider to control the weight of the brush
  this->brushWeightSlider = new QSlider(this);
  this->brushWeightSlider->setRange(0, 100);
  this->brushWeightSlider->setTickInterval(1);
  this->brushWeightSlider->setOrientation(Qt::Horizontal);
  this->brushWeightSlider->setValue(10);

  // Create a layout to hold the brush weight slider and its label
  QHBoxLayout *brushWeightLayout = new QHBoxLayout;
  brushWeightLayout->addWidget(new QLabel(tr("Brush Weight: ")));
  brushWeightLayout->addWidget(this->brushWeightSlider);

  // Add all the layouts and widgets to the main layout
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
  this->SetState(_toggle ? "raise" : std::string());
  this->lowerButton->setChecked(false);
  this->flattenButton->setChecked(false);
  this->courseButton->setChecked(false);
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnLower(bool _toggle)
{
  this->SetState(_toggle ? "lower" : std::string());
  this->raiseButton->setChecked(false);
  this->flattenButton->setChecked(false);
  this->courseButton->setChecked(false);
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnFlatten(bool _toggle)
{
  this->SetState(_toggle ? "flatten" : std::string());
  this->raiseButton->setChecked(false);
  this->lowerButton->setChecked(false);
  this->courseButton->setChecked(false);
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnCourse(bool _toggle)
{
  this->SetState(_toggle ? "course" : std::string());
  this->raiseButton->setChecked(false);
  this->lowerButton->setChecked(false);
  this->flattenButton->setChecked(false);
}

/////////////////////////////////////////////////
void TerrainEditorPalette::SetState(const std::string &_state)
{
  if (!_state.empty())
  {
    this->state = _state;

    // Add an event filter, which allows the TerrainEditor to capture
    // mouse events.
    MouseEventHandler::Instance()->AddPressFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMouseMove, this, _1));
  }
  else
  {
    this->state.clear();

    // Remove the event filters.
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

  // Get a filename to save to.
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
  {
    handled = this->Apply(_event, camera, heightmap);
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
  }
  else
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

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

  if (heightmap && !_event.shift)
  {
    if (_event.dragging)
      handled = this->Apply(_event, camera, heightmap);
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
  }
  else
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

  return handled;
}

/////////////////////////////////////////////////
bool TerrainEditorPalette::Apply(const common::MouseEvent &_event,
    rendering::CameraPtr _camera, rendering::Heightmap *_heightmap)
{
  bool handled = false;

  // Get the brush weight and size from the sliders.
  double brushWeight = this->brushWeightSlider->value() / 100.0;
  double brushSize = this->brushSizeSlider->value() / 100.0;

  if (this->state == "lower")
    handled = _heightmap->Lower(_camera, _event.pos, brushSize, brushWeight);
  else if (this->state == "raise")
    handled = _heightmap->Raise(_camera, _event.pos, brushSize, brushWeight);
  else if (this->state == "flatten")
    handled = _heightmap->Flatten(_camera, _event.pos, brushSize, brushWeight);
  else if (this->state == "rough")
    handled = _heightmap->Roughen(_camera, _event.pos, brushSize, brushWeight);

  return handled;
}
