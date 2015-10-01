/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
 * Brick-wall, Window and Door designed by Juan Pablo Bravo from the
 * thenounproject.com
 * Stairs designed by Brian Oppenlander from the thenounproject.com
*/
#include <boost/bind.hpp>

#include "gazebo/gui/building/BuildingEditorPalettePrivate.hh"
#include "gazebo/gui/building/BuildingEditorPalette.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/ImportImageDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingEditorPalette::BuildingEditorPalette(QWidget *_parent)
    : QWidget(_parent), dataPtr(new BuildingEditorPalettePrivate)
{
  this->setObjectName("buildingEditorPalette");

  this->dataPtr->buildingDefaultName = "Untitled";
  this->dataPtr->currentMode = std::string();

  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Model name layout
  QHBoxLayout *modelNameLayout = new QHBoxLayout;
  QLabel *modelLabel = new QLabel(tr("Model Name: "));
  this->dataPtr->modelNameEdit = new QLineEdit();
  this->dataPtr->modelNameEdit->setText(
      tr(this->dataPtr->buildingDefaultName.c_str()));
  modelNameLayout->addWidget(modelLabel);
  modelNameLayout->addWidget(this->dataPtr->modelNameEdit);
  connect(this->dataPtr->modelNameEdit, SIGNAL(textChanged(QString)), this,
          SLOT(OnNameChanged(QString)));

  // Brushes (button group)
  this->dataPtr->brushes = new QButtonGroup();
  connect(this->dataPtr->brushes, SIGNAL(buttonClicked(int)),
      this, SLOT(OnBrush(int)));

  QSize toolButtonSize(100, 100);
  QSize iconSize(65, 65);

  // Walls label
  QLabel *wallsLabel = new QLabel(tr(
       "<font size=4 color='white'>Create Walls</font>"));

  // Wall button
  QToolButton *wallButton = new QToolButton(this);
  wallButton->setFixedSize(toolButtonSize);
  wallButton->setCheckable(true);
  wallButton->setChecked(false);
  wallButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  wallButton->setIcon(QPixmap(":/images/wall.svg"));
  wallButton->setText("Wall");
  wallButton->setIconSize(QSize(iconSize));
  wallButton->setToolTip("Hold Shift to override snapping");
  connect(wallButton, SIGNAL(clicked()), this, SLOT(OnDrawWall()));

  // Features label
  QLabel *featuresLabel = new QLabel(tr(
       "<font size=4 color='white'>Add Features</font>"));

  // Window button
  QToolButton *windowButton = new QToolButton(this);
  windowButton->setFixedSize(toolButtonSize);
  windowButton->setCheckable(true);
  windowButton->setChecked(false);
  windowButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  windowButton->setIcon(QPixmap(":/images/window.svg"));
  windowButton->setText("Window");
  windowButton->setIconSize(QSize(iconSize));
  connect(windowButton, SIGNAL(clicked()), this, SLOT(OnAddWindow()));

  // Door button
  QToolButton *doorButton = new QToolButton(this);
  doorButton->setFixedSize(toolButtonSize);
  doorButton->setCheckable(true);
  doorButton->setChecked(false);
  doorButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  doorButton->setIcon(QPixmap(":/images/door.svg"));
  doorButton->setText("Door");
  doorButton->setIconSize(QSize(iconSize));
  connect(doorButton, SIGNAL(clicked()), this, SLOT(OnAddDoor()));

  // Stairs button
  QToolButton *stairsButton = new QToolButton(this);
  stairsButton->setFixedSize(toolButtonSize);
  stairsButton->setCheckable(true);
  stairsButton->setChecked(false);
  stairsButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  stairsButton->setIcon(QPixmap(":/images/stairs.svg"));
  stairsButton->setText("Stairs");
  stairsButton->setIconSize(QSize(iconSize));
  connect(stairsButton, SIGNAL(clicked()), this, SLOT(OnAddStair()));

  // Features layout
  QGridLayout *featuresLayout = new QGridLayout;
  featuresLayout->addWidget(windowButton, 0, 0);
  featuresLayout->addWidget(doorButton, 0, 1);
  featuresLayout->addWidget(stairsButton, 1, 0);

  // Colors
  QLabel *colorsLabel = new QLabel(tr(
      "<font size=4 color='white'>Add Color</font>"));

  QGridLayout *colorsLayout = new QGridLayout;
  this->dataPtr->colorList.push_back(QColor(255, 255, 255, 255));
  this->dataPtr->colorList.push_back(QColor(194, 169, 160, 255));
  this->dataPtr->colorList.push_back(QColor(235, 206, 157, 255));
  this->dataPtr->colorList.push_back(QColor(254, 121,   5, 255));
  this->dataPtr->colorList.push_back(QColor(255, 195,  78, 255));
  this->dataPtr->colorList.push_back(QColor(111, 203, 172, 255));
  for (unsigned int i = 0; i < this->dataPtr->colorList.size(); ++i)
  {
    QToolButton *colorButton = new QToolButton(this);
    colorButton->setFixedSize(40, 40);
    colorButton->setCheckable(true);
    colorButton->setChecked(false);
    colorButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
    QPixmap colorIcon(30, 30);
    colorIcon.fill(this->dataPtr->colorList.at(i));
    colorButton->setIcon(colorIcon);
    this->dataPtr->brushes->addButton(colorButton, i);
    colorsLayout->addWidget(colorButton, 0, i);
  }

  // Textures
  QLabel *texturesLabel = new QLabel(tr(
       "<font size=4 color='white'>Add Texture</font>"));

  QGridLayout *texturesLayout = new QGridLayout;
  std::vector<QString> textureButtonTextList;

  this->dataPtr->textureList.push_back(":wood.jpg");
  textureButtonTextList.push_back("Wood");
  this->dataPtr->textureList.push_back(":tiles.jpg");
  textureButtonTextList.push_back("Tiles");
  this->dataPtr->textureList.push_back(":bricks.png");
  textureButtonTextList.push_back("Bricks");

  for (unsigned int i = 0; i < this->dataPtr->textureList.size(); ++i)
  {
    QToolButton *textureButton = new QToolButton(this);
    textureButton->setFixedSize(70, 70);
    textureButton->setCheckable(true);
    textureButton->setChecked(false);
    textureButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    textureButton->setIcon(QPixmap(this->dataPtr->textureList[i]).scaled(
        QSize(90, 90), Qt::IgnoreAspectRatio));
    textureButton->setText(textureButtonTextList[i]);
    textureButton->setIconSize(QSize(40, 40));
    this->dataPtr->brushes->addButton(textureButton,
        this->dataPtr->brushes->buttons().size());
    texturesLayout->addWidget(textureButton, 0, i);
  }

  // Import button
  QPushButton *importImageButton = new QPushButton(tr("Import"),
      this);
  importImageButton->setCheckable(true);
  importImageButton->setChecked(false);
  importImageButton->setToolTip(tr(
      "Import an existing floor plan to use as a guide"));
  connect(importImageButton, SIGNAL(clicked()), this, SLOT(OnImportImage()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(importImageButton);
  buttonsLayout->setAlignment(Qt::AlignHCenter);
  buttonsLayout->setContentsMargins(30, 11, 30, 11);

  // Main layout
  mainLayout->addLayout(modelNameLayout);
  mainLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  mainLayout->addWidget(wallsLabel);
  mainLayout->addWidget(wallButton);
  mainLayout->addWidget(featuresLabel);
  mainLayout->addLayout(featuresLayout);
  mainLayout->addWidget(colorsLabel);
  mainLayout->addLayout(colorsLayout);
  mainLayout->addWidget(texturesLabel);
  mainLayout->addLayout(texturesLayout);
  mainLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  mainLayout->addLayout(buttonsLayout);
  mainLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);

  this->setLayout(mainLayout);

  // Connections
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectSaveBuildingModel(
      boost::bind(&BuildingEditorPalette::OnSaveModel, this, _1, _2)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectNewBuildingModel(
      boost::bind(&BuildingEditorPalette::OnNewModel, this)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectCreateBuildingEditorItem(
      boost::bind(&BuildingEditorPalette::OnCreateEditorItem, this, _1)));

  // All buttons must be added after the color and texture buttons
  this->dataPtr->brushes->addButton(wallButton,
      this->dataPtr->brushes->buttons().size());
  this->dataPtr->brushes->addButton(windowButton,
      this->dataPtr->brushes->buttons().size());
  this->dataPtr->brushes->addButton(doorButton,
      this->dataPtr->brushes->buttons().size());
  this->dataPtr->brushes->addButton(stairsButton,
      this->dataPtr->brushes->buttons().size());
  this->dataPtr->brushes->addButton(importImageButton,
      this->dataPtr->brushes->buttons().size());
}

/////////////////////////////////////////////////
BuildingEditorPalette::~BuildingEditorPalette()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
std::string BuildingEditorPalette::GetModelName() const
{
  return this->dataPtr->modelNameEdit->text().toStdString();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDrawWall()
{
  if (this->dataPtr->currentMode != "wall")
    gui::editor::Events::createBuildingEditorItem("wall");
  else
    gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddWindow()
{
  if (this->dataPtr->currentMode != "window")
    gui::editor::Events::createBuildingEditorItem("window");
  else
    gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddDoor()
{
  if (this->dataPtr->currentMode != "door")
    gui::editor::Events::createBuildingEditorItem("door");
  else
    gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnImportImage()
{
  if (this->dataPtr->currentMode != "image")
    gui::editor::Events::createBuildingEditorItem("image");
  else
    gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddStair()
{
  if (this->dataPtr->currentMode != "stairs")
    gui::editor::Events::createBuildingEditorItem("stairs");
  else
    gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnNewModel()
{
  this->dataPtr->modelNameEdit->setText(
      tr(this->dataPtr->buildingDefaultName.c_str()));
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnSaveModel(const std::string &_saveName,
    const std::string &/*_saveLocation*/)
{
  this->dataPtr->modelNameEdit->setText(tr(_saveName.c_str()));
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnNameChanged(const QString &_name)
{
  gui::editor::Events::buildingNameChanged(_name.toStdString());
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnCreateEditorItem(const std::string &_mode)
{
  gui::editor::Events::colorSelected(QColor::Invalid);
  gui::editor::Events::textureSelected(QString(""));

  if (_mode.empty() || this->dataPtr->currentMode == _mode)
  {
    this->dataPtr->brushes->setExclusive(false);
    if (this->dataPtr->brushes->checkedButton())
      this->dataPtr->brushes->checkedButton()->setChecked(false);
    this->dataPtr->brushes->setExclusive(true);

    this->dataPtr->currentMode = std::string();
  }
  else
  {
    this->dataPtr->currentMode = _mode;
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnBrush(int _buttonId)
{
  if (_buttonId < static_cast<int>(this->dataPtr->colorList.size()))
  {
    this->OnColor(_buttonId);
  }
  else if (_buttonId < static_cast<int>(this->dataPtr->colorList.size()) +
                       static_cast<int>(this->dataPtr->textureList.size()))
  {
    this->OnTexture(_buttonId -
        static_cast<int>(this->dataPtr->colorList.size()));
  }
  else
  {
    gzwarn << "Brushes other than color and texture are handled elsewhere."
           << std::endl;
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnColor(int _buttonId)
{
  std::ostringstream colorStr;
  colorStr << "color_" << _buttonId;
  QColor color = this->dataPtr->colorList[_buttonId];
  if (this->dataPtr->currentMode != colorStr.str())
  {
    gui::editor::Events::colorSelected(color);
    this->dataPtr->currentMode = colorStr.str();

    QPixmap colorCursor(30, 30);
    colorCursor.fill(color);
    QApplication::setOverrideCursor(QCursor(colorCursor));
  }
  else
  {
    gui::editor::Events::createBuildingEditorItem(std::string());
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnTexture(int _buttonId)
{
  std::ostringstream textureStr;
  textureStr << "texture_" << _buttonId;
  QString texture = this->dataPtr->textureList[_buttonId];
  if (this->dataPtr->currentMode != textureStr.str())
  {
    gui::editor::Events::textureSelected(texture);
    this->dataPtr->currentMode = textureStr.str();

    QPixmap textureCursor(this->dataPtr->textureList[_buttonId]);
    textureCursor = textureCursor.scaled(QSize(30, 30), Qt::IgnoreAspectRatio,
        Qt::SmoothTransformation);
    QApplication::setOverrideCursor(textureCursor);
  }
  else
  {
    gui::editor::Events::createBuildingEditorItem(std::string());
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::mousePressEvent(QMouseEvent * /*_event*/)
{
  // Cancel draw mode
  gui::editor::Events::createBuildingEditorItem(std::string());
}
