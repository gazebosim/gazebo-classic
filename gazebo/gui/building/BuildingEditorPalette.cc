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
  this->dataPtr->brushIdToModeMap["wall"] =
    this->dataPtr->brushes->buttons().size();
  this->dataPtr->brushes->addButton(wallButton,
      this->dataPtr->brushes->buttons().size());

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
  this->dataPtr->brushIdToModeMap["window"] =
    this->dataPtr->brushes->buttons().size();
  this->dataPtr->brushes->addButton(windowButton,
      this->dataPtr->brushes->buttons().size());

  // Door button
  QToolButton *doorButton = new QToolButton(this);
  doorButton->setFixedSize(toolButtonSize);
  doorButton->setCheckable(true);
  doorButton->setChecked(false);
  doorButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  doorButton->setIcon(QPixmap(":/images/door.svg"));
  doorButton->setText("Door");
  doorButton->setIconSize(QSize(iconSize));
  this->dataPtr->brushIdToModeMap["door"] =
    this->dataPtr->brushes->buttons().size();
  this->dataPtr->brushes->addButton(doorButton,
      this->dataPtr->brushes->buttons().size());

  // Stairs button
  QToolButton *stairsButton = new QToolButton(this);
  stairsButton->setFixedSize(toolButtonSize);
  stairsButton->setCheckable(true);
  stairsButton->setChecked(false);
  stairsButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  stairsButton->setIcon(QPixmap(":/images/stairs.svg"));
  stairsButton->setText("Stairs");
  stairsButton->setIconSize(QSize(iconSize));
  this->dataPtr->brushIdToModeMap["stairs"] =
    this->dataPtr->brushes->buttons().size();
  this->dataPtr->brushes->addButton(stairsButton,
      this->dataPtr->brushes->buttons().size());

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
    std::ostringstream colorStr;
    colorStr << "color_" << i;
    this->dataPtr->lastDefaultColor = colorStr.str();
    this->dataPtr->brushIdToModeMap[this->dataPtr->lastDefaultColor] =
        this->dataPtr->brushes->buttons().size();
    this->dataPtr->brushes->addButton(colorButton,
        this->dataPtr->brushes->buttons().size());
    colorsLayout->addWidget(colorButton, 0, i);
  }

  this->dataPtr->customColorButton = new QPushButton("More");
  this->dataPtr->customColorButton->setCheckable(true);
  this->dataPtr->customColorButton->setChecked(false);
  colorsLayout->addWidget(this->dataPtr->customColorButton, 1, 4, 1, 2);
  this->dataPtr->brushIdToModeMap["color_custom"] =
    this->dataPtr->brushes->buttons().size();
  this->dataPtr->brushes->addButton(this->dataPtr->customColorButton,
      this->dataPtr->brushes->buttons().size());

  this->dataPtr->customColorDialog = new QColorDialog(Qt::green, this);
  this->dataPtr->customColorDialog->setWindowModality(Qt::NonModal);
  connect(this->dataPtr->customColorDialog,
      SIGNAL(currentColorChanged(const QColor)),
      this, SLOT(OnCustomColor(const QColor)));
  connect(this->dataPtr->customColorDialog, SIGNAL(rejected()), this,
      SLOT(CancelDrawModes()));

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
    std::ostringstream textureStr;
    textureStr << "texture_" << i;
    this->dataPtr->lastDefaultTexture = textureStr.str();
    this->dataPtr->brushIdToModeMap[this->dataPtr->lastDefaultTexture] =
        this->dataPtr->brushes->buttons().size();
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
  this->dataPtr->brushIdToModeMap["image"] =
    this->dataPtr->brushes->buttons().size();
  this->dataPtr->brushes->addButton(importImageButton,
      this->dataPtr->brushes->buttons().size());

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

  // Scroll area
  auto scrollWidget = new QWidget();
  scrollWidget->setObjectName("buildingEditorPaletteScroll");
  scrollWidget->setLayout(mainLayout);
  scrollWidget->setMinimumWidth(200);

  auto scrollArea = new QScrollArea;
  scrollArea->setWidget(scrollWidget);
  scrollArea->setWidgetResizable(true);

  auto scrollLayout = new QVBoxLayout();
  scrollLayout->addWidget(scrollArea);

  this->setLayout(scrollLayout);

  // Connections
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectSaveBuildingModel(
      boost::bind(&BuildingEditorPalette::OnSaveModel, this, _1)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectNewBuildingModel(
      boost::bind(&BuildingEditorPalette::OnNewModel, this)));

  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectCreateBuildingEditorItem(
      boost::bind(&BuildingEditorPalette::OnCreateEditorItem, this, _1)));
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
void BuildingEditorPalette::OnBrush(int _buttonId)
{
  if (_buttonId == this->dataPtr->brushIdToModeMap["wall"])
  {
    this->OnDrawWall();
  }
  else if (_buttonId == this->dataPtr->brushIdToModeMap["window"])
  {
    this->OnAddWindow();
  }
  else if (_buttonId == this->dataPtr->brushIdToModeMap["door"])
  {
    this->OnAddDoor();
  }
  else if (_buttonId == this->dataPtr->brushIdToModeMap["stairs"])
  {
    this->OnAddStair();
  }
  else if (_buttonId >= this->dataPtr->brushIdToModeMap["color_0"] &&
           _buttonId <= this->dataPtr->brushIdToModeMap[
           this->dataPtr->lastDefaultColor])
  {
    this->OnDefaultColor(_buttonId -
        this->dataPtr->brushIdToModeMap["color_0"]);
  }
  else if (_buttonId == this->dataPtr->brushIdToModeMap["color_custom"])
  {
    this->OnCustomColorDialog();
  }
  else if (_buttonId >= this->dataPtr->brushIdToModeMap["texture_0"] &&
           _buttonId <= this->dataPtr->brushIdToModeMap[
           this->dataPtr->lastDefaultTexture])
  {
    this->OnTexture(_buttonId - this->dataPtr->brushIdToModeMap["texture_0"]);
  }
  else if (_buttonId == this->dataPtr->brushIdToModeMap["image"])
  {
    this->OnImportImage();
  }
  else
  {
    gzerr << "Requested brush doesn't exist." << std::endl;
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDrawWall()
{
  if (this->dataPtr->currentMode != "wall")
    gui::editor::Events::createBuildingEditorItem("wall");
  else
    this->CancelDrawModes();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddWindow()
{
  if (this->dataPtr->currentMode != "window")
    gui::editor::Events::createBuildingEditorItem("window");
  else
    this->CancelDrawModes();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddDoor()
{
  if (this->dataPtr->currentMode != "door")
    gui::editor::Events::createBuildingEditorItem("door");
  else
    this->CancelDrawModes();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnImportImage()
{
  if (this->dataPtr->currentMode != "image")
    gui::editor::Events::createBuildingEditorItem("image");
  else
    this->CancelDrawModes();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnAddStair()
{
  if (this->dataPtr->currentMode != "stairs")
    gui::editor::Events::createBuildingEditorItem("stairs");
  else
    this->CancelDrawModes();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnNewModel()
{
  this->dataPtr->modelNameEdit->setText(
      tr(this->dataPtr->buildingDefaultName.c_str()));
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnSaveModel(const std::string &_saveName)
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

    this->dataPtr->currentMode.clear();
  }
  else
  {
    this->dataPtr->currentMode = _mode;
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnDefaultColor(int _colorId)
{
  std::ostringstream colorStr;
  colorStr << "color_" << _colorId;
  QColor color = this->dataPtr->colorList[_colorId];
  if (this->dataPtr->currentMode != colorStr.str())
  {
    this->dataPtr->currentMode = colorStr.str();
    this->OnColor(color);
  }
  else
  {
    this->CancelDrawModes();
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnCustomColorDialog()
{
  this->CancelDrawModes();
  this->dataPtr->customColorButton->setChecked(true);
  this->dataPtr->customColorDialog->show();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnCustomColor(const QColor _color)
{
  this->dataPtr->customColorButton->setChecked(true);
  if (_color.isValid())
  {
    this->dataPtr->currentMode = "color_custom";
    this->OnColor(_color);
  }
  else
  {
    this->CancelDrawModes();
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnColor(QColor _color)
{
  gui::editor::Events::colorSelected(_color);
  QPixmap colorCursor(30, 30);
  colorCursor.fill(_color);
  QApplication::setOverrideCursor(QCursor(colorCursor));
}

/////////////////////////////////////////////////
void BuildingEditorPalette::OnTexture(int _textureId)
{
  std::ostringstream textureStr;
  textureStr << "texture_" << _textureId;
  QString texture = this->dataPtr->textureList[_textureId];
  if (this->dataPtr->currentMode != textureStr.str())
  {
    gui::editor::Events::textureSelected(texture);
    this->dataPtr->currentMode = textureStr.str();

    QPixmap textureCursor(this->dataPtr->textureList[_textureId]);
    textureCursor = textureCursor.scaled(QSize(30, 30), Qt::IgnoreAspectRatio,
        Qt::SmoothTransformation);
    QApplication::setOverrideCursor(textureCursor);
  }
  else
  {
    this->CancelDrawModes();
  }
}

/////////////////////////////////////////////////
void BuildingEditorPalette::mousePressEvent(QMouseEvent * /*_event*/)
{
  this->CancelDrawModes();
}

/////////////////////////////////////////////////
void BuildingEditorPalette::CancelDrawModes()
{
  gui::editor::Events::createBuildingEditorItem(std::string());
}

/////////////////////////////////////////////////
QColorDialog *BuildingEditorPalette::CustomColorDialog() const
{
  return this->dataPtr->customColorDialog;
}
