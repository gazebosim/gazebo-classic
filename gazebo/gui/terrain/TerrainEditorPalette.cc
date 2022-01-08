/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/bind/bind.hpp>

#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/terrain/TerrainEditorPalette.hh"
#include "gazebo/gui/terrain/TerrainEditorPalettePrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TerrainEditorPalette::TerrainEditorPalette(QWidget *_parent)
    : QWidget(_parent),
      dataPtr(new TerrainEditorPalettePrivate())
{
  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Create the button to raise terrain
  QPushButton *raiseButton = new QPushButton("Raise", this);
  raiseButton->setStatusTip(tr("Left-mouse press to raise terrain."));
  raiseButton->setCheckable(true);
  raiseButton->setChecked(false);
  connect(raiseButton, SIGNAL(toggled(bool)), this, SLOT(OnRaise(bool)));

  // Create the button to lower terrain
  QPushButton *lowerButton = new QPushButton("Lower", this);
  lowerButton->setStatusTip(tr("Left-mouse press to lower terrain."));
  lowerButton->setCheckable(true);
  lowerButton->setChecked(false);
  connect(lowerButton, SIGNAL(toggled(bool)), this, SLOT(OnLower(bool)));

  // Create the button to flatten terrain
  QPushButton *flattenButton = new QPushButton("Flatten", this);
  flattenButton->setStatusTip(tr("Left-mouse press to flatten terrain."));
  flattenButton->setCheckable(true);
  flattenButton->setChecked(false);
  connect(flattenButton, SIGNAL(toggled(bool)), this, SLOT(OnFlatten(bool)));

  // Create the button to roughen terrain
  QPushButton *heightButton = new QPushButton("Pick Height", this);
  heightButton->setStatusTip(
      tr("Left-mouse press to select a terrain height."));
  heightButton->setCheckable(true);
  heightButton->setChecked(false);
  connect(heightButton, SIGNAL(toggled(bool)), this, SLOT(OnPickHeight(bool)));

  QButtonGroup *buttonGroup = new QButtonGroup;
  buttonGroup->addButton(raiseButton);
  buttonGroup->addButton(lowerButton);
  buttonGroup->addButton(flattenButton);
  buttonGroup->addButton(heightButton);

  // Create the layout to hold all the buttons
  QGridLayout *buttonLayout = new QGridLayout;
  buttonLayout->addWidget(raiseButton, 0, 0);
  buttonLayout->addWidget(lowerButton, 0, 1);
  buttonLayout->addWidget(flattenButton, 1, 0);
  buttonLayout->addWidget(heightButton, 1, 1);

  // Add a save button
  QPushButton *saveButton = new QPushButton("Save Image", this);
  saveButton->setStatusTip(tr("Save terrain as a PNG."));
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));

  // Create a slider to control the outer size of the brush
  this->dataPtr->outsideRadiusSlider = new QSlider(this);
  this->dataPtr->outsideRadiusSlider->setRange(1, 100000);
  this->dataPtr->outsideRadiusSlider->setTickInterval(1);
  this->dataPtr->outsideRadiusSlider->setOrientation(Qt::Horizontal);
  this->dataPtr->outsideRadiusSlider->setValue(10);
  connect(this->dataPtr->outsideRadiusSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnOutsideRadiusSlider(int)));

  this->dataPtr->outsideRadiusSpin = new QDoubleSpinBox(this);
  this->dataPtr->outsideRadiusSpin->setRange(0, 1.0);
  this->dataPtr->outsideRadiusSpin->setSingleStep(0.001);
  this->dataPtr->outsideRadiusSpin->setDecimals(3);
  connect(this->dataPtr->outsideRadiusSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnOutsideRadiusSpin(double)));

  // Create a layout to hold the outer brush size slider and its label
  QHBoxLayout *outsideRadiusSpinLayout = new QHBoxLayout;
  outsideRadiusSpinLayout->addWidget(new QLabel(tr("Outside radius: ")));
  outsideRadiusSpinLayout->addStretch(2);
  outsideRadiusSpinLayout->addWidget(this->dataPtr->outsideRadiusSpin);

  QVBoxLayout *outsideRadiusLayout = new QVBoxLayout;
  outsideRadiusLayout->addLayout(outsideRadiusSpinLayout);
  outsideRadiusLayout->addWidget(this->dataPtr->outsideRadiusSlider);


  // Create a slider to control the inner size of the brush
  this->dataPtr->insideRadiusSlider = new QSlider(this);
  this->dataPtr->insideRadiusSlider->setRange(0, 100000);
  this->dataPtr->insideRadiusSlider->setTickInterval(1);
  this->dataPtr->insideRadiusSlider->setOrientation(Qt::Horizontal);
  this->dataPtr->insideRadiusSlider->setValue(10);
  connect(this->dataPtr->insideRadiusSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnInsideRadiusSlider(int)));

  this->dataPtr->insideRadiusSpin = new QDoubleSpinBox(this);
  this->dataPtr->insideRadiusSpin->setRange(0, 1.0);
  this->dataPtr->insideRadiusSpin->setSingleStep(0.001);
  this->dataPtr->insideRadiusSpin->setDecimals(3);
  connect(this->dataPtr->insideRadiusSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnInsideRadiusSpin(double)));

  // Create a layout to hold the inner brush size slider and its label
  QHBoxLayout *insideRadiusSpinLayout = new QHBoxLayout;
  insideRadiusSpinLayout->addWidget(new QLabel(tr("Inside radius: ")));
  insideRadiusSpinLayout->addStretch(2);
  insideRadiusSpinLayout->addWidget(this->dataPtr->insideRadiusSpin);

  QVBoxLayout *insideRadiusLayout = new QVBoxLayout;
  insideRadiusLayout->addLayout(insideRadiusSpinLayout);
  insideRadiusLayout->addWidget(this->dataPtr->insideRadiusSlider);

  // Create a slider to control the weight of the brush
  this->dataPtr->weightSlider = new QSlider(this);
  this->dataPtr->weightSlider->setRange(1, 10000);
  this->dataPtr->weightSlider->setTickInterval(1);
  this->dataPtr->weightSlider->setOrientation(Qt::Horizontal);
  this->dataPtr->weightSlider->setValue(10);
  connect(this->dataPtr->weightSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnWeightSlider(int)));

  this->dataPtr->weightSpin = new QDoubleSpinBox(this);
  this->dataPtr->weightSpin->setRange(.01, 1.0);
  this->dataPtr->weightSpin->setSingleStep(.1);
  connect(this->dataPtr->weightSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnWeightSpin(double)));


  // Create a layout to hold the brush weight slider and its label
  QHBoxLayout *weightSpinLayout = new QHBoxLayout;
  weightSpinLayout->addWidget(new QLabel(tr("Weight: ")));
  weightSpinLayout->addStretch(2);
  weightSpinLayout->addWidget(this->dataPtr->weightSpin);

  QVBoxLayout *weightLayout = new QVBoxLayout;
  weightLayout->addLayout(weightSpinLayout);
  weightLayout->addWidget(this->dataPtr->weightSlider);


  // Create a slider to control the weight of the brush
  this->dataPtr->heightSlider = new QSlider(this);
  this->dataPtr->heightSlider->setRange(1, 100);
  this->dataPtr->heightSlider->setTickInterval(1);
  this->dataPtr->heightSlider->setOrientation(Qt::Horizontal);
  this->dataPtr->heightSlider->setValue(10);
  connect(this->dataPtr->heightSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnHeightSlider(int)));

  this->dataPtr->heightSpin = new QDoubleSpinBox(this);
  connect(this->dataPtr->heightSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnHeightSpin(double)));

  // Create a layout to hold the brush height slider and its label
  QHBoxLayout *heightSpinLayout = new QHBoxLayout;
  heightSpinLayout->addWidget(new QLabel(tr("Height: ")));
  heightSpinLayout->addStretch(2);
  heightSpinLayout->addWidget(this->dataPtr->heightSpin);

  QVBoxLayout *heightLayout = new QVBoxLayout;
  heightLayout->setContentsMargins(0, 0, 0, 0);
  heightLayout->addLayout(heightSpinLayout);
  heightLayout->addWidget(this->dataPtr->heightSlider);


  // Add all the layouts and widgets to the main layout
  mainLayout->addLayout(buttonLayout);
  mainLayout->addLayout(outsideRadiusLayout);
  mainLayout->addLayout(insideRadiusLayout);
  mainLayout->addLayout(weightLayout);
  mainLayout->addLayout(heightLayout);
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
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnLower(bool _toggle)
{
  this->SetState(_toggle ? "lower" : std::string());
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnFlatten(bool _toggle)
{
  this->SetState(_toggle ? "flatten" : std::string());
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnPickHeight(bool /*_toggle*/)
{
}

/////////////////////////////////////////////////
void TerrainEditorPalette::SetState(const std::string &_state)
{
  if (!_state.empty())
  {
    this->dataPtr->state = _state;

    // Add an event filter, which allows the TerrainEditor to capture
    // mouse events.
    using namespace boost::placeholders;
    MouseEventHandler::Instance()->AddPressFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("terrain",
        boost::bind(&TerrainEditorPalette::OnMouseMove, this, _1));
  }
  else
  {
    this->dataPtr->state.clear();

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
  common::Image img = heightmap->Image();

  // Get a filename to save to.
  // Note that file dialog static functions seem to be broken (issue #1514)
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
  if (_event.Button() != common::MouseEvent::LEFT)
    return false;

  bool handled = false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  // Get a pointer to the heightmap, if the scen is valid.
  rendering::Heightmap *heightmap = scene ? scene->GetHeightmap() : NULL;

  // Only try to modify if the heightmap exists, and the LEFT mouse button
  // was used.
  if (heightmap && !_event.Shift())
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
  if (_event.Button() != common::MouseEvent::LEFT)
    return false;

  bool handled = false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  // Get a pointer to the heightmap, if the scen is valid.
  rendering::Heightmap *heightmap = scene ? scene->GetHeightmap() : NULL;

  if (heightmap && !_event.Shift())
  {
    if (_event.Dragging())
      handled = this->Apply(_event, camera, heightmap);
    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));
  }
  else
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));

  return handled;
}

/////////////////////////////////////////////////
bool TerrainEditorPalette::Apply(const common::MouseEvent &_event,
    rendering::CameraPtr _camera, rendering::Heightmap *_heightmap) const
{
  bool handled = false;

  // Get the brush weight and size from the sliders.
  double weight = this->dataPtr->weightSpin->value();
  double outsideRadius = this->dataPtr->outsideRadiusSpin->value() / 10.0;
  double insideRadius = this->dataPtr->insideRadiusSpin->value() / 10.0;

  if (this->dataPtr->state == "lower")
  {
    handled = _heightmap->Lower(_camera, _event.Pos(), outsideRadius,
        insideRadius, weight);
  }
  else if (this->dataPtr->state == "raise")
  {
    handled = _heightmap->Raise(_camera, _event.Pos(), outsideRadius,
        insideRadius, weight);
  }
  else if (this->dataPtr->state == "flatten")
  {
    handled = _heightmap->Flatten(_camera, _event.Pos(), outsideRadius,
        insideRadius, weight);
  }
  else if (this->dataPtr->state == "smooth")
  {
    handled = _heightmap->Smooth(_camera, _event.Pos(), outsideRadius,
        insideRadius, weight);
  }

  return handled;
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnOutsideRadiusSpin(double _value)
{
  disconnect(this->dataPtr->outsideRadiusSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnOutsideRadiusSlider(int)));

  this->dataPtr->outsideRadiusSlider->setValue(
      static_cast<int>(rint(
        _value * this->dataPtr->outsideRadiusSlider->maximum())));

  connect(this->dataPtr->outsideRadiusSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnOutsideRadiusSlider(int)));
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnOutsideRadiusSlider(int _value)
{
  disconnect(this->dataPtr->outsideRadiusSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnOutsideRadiusSpin(double)));

  this->dataPtr->outsideRadiusSpin->setValue(_value /
      static_cast<double>(this->dataPtr->outsideRadiusSlider->maximum()));

  connect(this->dataPtr->outsideRadiusSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnOutsideRadiusSpin(double)));
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnInsideRadiusSpin(double _value)
{
  disconnect(this->dataPtr->insideRadiusSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnInsideRadiusSlider(int)));

  this->dataPtr->insideRadiusSlider->setValue(
      static_cast<int>(rint(
        _value * this->dataPtr->insideRadiusSlider->maximum())));

  connect(this->dataPtr->insideRadiusSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnInsideRadiusSlider(int)));
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnInsideRadiusSlider(int _value)
{
  disconnect(this->dataPtr->insideRadiusSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnInsideRadiusSpin(double)));


  this->dataPtr->insideRadiusSpin->setValue(_value /
      static_cast<double>(this->dataPtr->insideRadiusSlider->maximum()));

  connect(this->dataPtr->insideRadiusSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnInsideRadiusSpin(double)));
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnWeightSpin(double _value)
{
  disconnect(this->dataPtr->weightSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnWeightSlider(int)));

  this->dataPtr->weightSlider->setValue(
      static_cast<int>(rint(_value * this->dataPtr->weightSlider->maximum())));

  connect(this->dataPtr->weightSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnWeightSlider(int)));
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnWeightSlider(int _value)
{
  disconnect(this->dataPtr->weightSpin, SIGNAL(valueChanged(double)),
             this, SLOT(OnWeightSpin(double)));

  this->dataPtr->weightSpin->setValue(_value /
      static_cast<double>(this->dataPtr->weightSlider->maximum()));

  connect(this->dataPtr->weightSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnWeightSpin(double)));
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnHeightSpin(double _value)
{
  disconnect(this->dataPtr->heightSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnHeightSlider(int)));

  this->dataPtr->heightSlider->setValue(static_cast<int>(rint(_value)));

  connect(this->dataPtr->heightSlider, SIGNAL(valueChanged(int)),
      this, SLOT(OnHeightSlider(int)));
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnHeightSlider(int _value)
{
  disconnect(this->dataPtr->heightSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnHeightSpin(double)));

  this->dataPtr->heightSpin->setValue(_value / 10.0);

  connect(this->dataPtr->heightSpin, SIGNAL(valueChanged(double)),
      this, SLOT(OnHeightSpin(double)));
}
