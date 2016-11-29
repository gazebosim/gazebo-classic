/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/KeyEvent.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/model/ExtrudeDialog.hh"
#include "gazebo/gui/model/ImportDialog.hh"
#include "gazebo/gui/model/ModelEditorPalette.hh"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \class ModelEditorPalette ModelEditorPalette.hh
    /// \brief Private data for the ModelEditorPalette class.
    class ModelEditorPalettePrivate
    {
      /// \brief Links button group.
      public: QButtonGroup *linkButtonGroup;

      /// \brief Model creator.
      public: ModelCreator *modelCreator;

      /// \brief Layout for other items in the palette.
      public: QVBoxLayout *otherItemsLayout;

      /// \brief Map of categories to their layout
      public: std::map<std::string, QGridLayout *> categories;

      /// \brief Vertical splitter between widgets.
      public: QSplitter *splitter;
    };
  }
}

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelEditorPalette::ModelEditorPalette(QWidget *_parent)
    : QWidget(_parent), dataPtr(new ModelEditorPalettePrivate)
{
  this->setObjectName("modelEditorPalette");

  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Simple Shapes
  QLabel *shapesLabel = new QLabel(tr(
       "<font size=4 color='white'>Simple Shapes</font>"));

  QHBoxLayout *shapesLayout = new QHBoxLayout;

  QSize toolButtonSize(70, 70);
  QSize iconSize(40, 40);

  // Cylinder button
  QToolButton *cylinderButton = new QToolButton(this);
  cylinderButton->setObjectName("modelEditorPaletteCylinderButton");
  cylinderButton->setFixedSize(toolButtonSize);
  cylinderButton->setToolTip(tr("Cylinder"));
  cylinderButton->setIcon(QPixmap(":/images/cylinder.png"));
  cylinderButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  cylinderButton->setIconSize(QSize(iconSize));
  cylinderButton->setCheckable(true);
  cylinderButton->setChecked(false);
  connect(cylinderButton, SIGNAL(clicked()), this, SLOT(OnCylinder()));
  shapesLayout->addWidget(cylinderButton);

  // Sphere button
  QToolButton *sphereButton = new QToolButton(this);
  sphereButton->setObjectName("modelEditorPaletteSphereButton");
  sphereButton->setFixedSize(toolButtonSize);
  sphereButton->setToolTip(tr("Sphere"));
  sphereButton->setIcon(QPixmap(":/images/sphere.png"));
  sphereButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  sphereButton->setIconSize(QSize(iconSize));
  sphereButton->setCheckable(true);
  sphereButton->setChecked(false);
  connect(sphereButton, SIGNAL(clicked()), this, SLOT(OnSphere()));
  shapesLayout->addWidget(sphereButton);

  // Box button
  QToolButton *boxButton = new QToolButton(this);
  boxButton->setObjectName("modelEditorPaletteBoxButton");
  boxButton->setFixedSize(toolButtonSize);
  boxButton->setToolTip(tr("Box"));
  boxButton->setIcon(QPixmap(":/images/box.png"));
  boxButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  boxButton->setIconSize(QSize(iconSize));
  boxButton->setCheckable(true);
  boxButton->setChecked(false);
  connect(boxButton, SIGNAL(clicked()), this, SLOT(OnBox()));
  shapesLayout->addWidget(boxButton);

  // Custom Shapes
  QLabel *customShapesLabel = new QLabel(tr(
       "<font size=4 color='white'>Custom Shapes</font>"));

  QHBoxLayout *customLayout = new QHBoxLayout;
  customLayout->setAlignment(Qt::AlignLeft);
  customLayout->addItem(new QSpacerItem(30, 30, QSizePolicy::Minimum,
      QSizePolicy::Minimum));

  QPushButton *customButton = new QPushButton(tr("Add"), this);
  customButton->setMaximumWidth(60);
  customButton->setCheckable(true);
  customButton->setChecked(false);
  connect(customButton, SIGNAL(clicked()), this, SLOT(OnCustom()));
  customLayout->addWidget(customButton, 0, 0);

  // Button group
  this->dataPtr->linkButtonGroup = new QButtonGroup;
  this->dataPtr->linkButtonGroup->addButton(cylinderButton);
  this->dataPtr->linkButtonGroup->addButton(sphereButton);
  this->dataPtr->linkButtonGroup->addButton(boxButton);
  this->dataPtr->linkButtonGroup->addButton(customButton);

  this->dataPtr->modelCreator = new gui::ModelCreator(this);
  connect(this->dataPtr->modelCreator, SIGNAL(LinkAdded()), this,
      SLOT(OnLinkAdded()));

  this->dataPtr->otherItemsLayout = new QVBoxLayout();
  this->dataPtr->otherItemsLayout->setContentsMargins(0, 0, 0, 0);

  // Palette layout
  QVBoxLayout *paletteLayout = new QVBoxLayout();
  paletteLayout->addWidget(shapesLabel);
  paletteLayout->addLayout(shapesLayout);
  paletteLayout->addWidget(customShapesLabel);
  paletteLayout->addLayout(customLayout);
  paletteLayout->addLayout(this->dataPtr->otherItemsLayout);
  paletteLayout->addItem(new QSpacerItem(30, 30, QSizePolicy::Minimum,
      QSizePolicy::Minimum));
  paletteLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
  QWidget *paletteWidget = new QWidget();
  paletteWidget->setLayout(paletteLayout);

  // Main layout
  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;

  this->dataPtr->splitter = new QSplitter(Qt::Vertical, this);
  this->dataPtr->splitter->addWidget(paletteWidget);
  this->dataPtr->splitter->setCollapsible(0, false);

  frameLayout->addWidget(this->dataPtr->splitter);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);

  mainLayout->addWidget(frame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  KeyEventHandler::Instance()->AddPressFilter("model_editor",
    std::bind(&ModelEditorPalette::OnKeyPress, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
ModelEditorPalette::~ModelEditorPalette()
{
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnCylinder()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->dataPtr->modelCreator->AddLink(ModelCreator::ENTITY_CYLINDER);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSphere()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->dataPtr->modelCreator->AddLink(ModelCreator::ENTITY_SPHERE);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnBox()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->dataPtr->modelCreator->AddLink(ModelCreator::ENTITY_BOX);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnCustom()
{
  ImportDialog importDialog(this);
  importDialog.deleteLater();
  if (importDialog.exec() == QDialog::Accepted)
  {
    QFileInfo info(QString::fromStdString(importDialog.GetImportPath()));
    if (info.isFile())
    {
      event::Events::setSelectedEntity("", "normal");
      g_arrowAct->trigger();
      if (info.completeSuffix().toLower() == "dae" ||
          info.completeSuffix().toLower() == "stl")
      {
        this->dataPtr->modelCreator->AddCustomLink(ModelCreator::ENTITY_MESH,
            ignition::math::Vector3d::One, ignition::math::Pose3d::Zero,
            importDialog.GetImportPath());
      }
      else if (info.completeSuffix().toLower() == "svg")
      {
        ExtrudeDialog extrudeDialog(importDialog.GetImportPath(), this);
        extrudeDialog.deleteLater();
        if (extrudeDialog.exec() == QDialog::Accepted)
        {
          this->dataPtr->modelCreator->AddCustomLink(
              ModelCreator::ENTITY_POLYLINE,
              ignition::math::Vector3d(1.0/extrudeDialog.GetResolution(),
              1.0/extrudeDialog.GetResolution(),
              extrudeDialog.GetThickness()),
              ignition::math::Pose3d::Zero, importDialog.GetImportPath(),
              extrudeDialog.GetSamples());
        }
        else
        {
          this->OnCustom();
        }
      }
    }
  }
  else
  {
    // this unchecks the custom button
    this->OnLinkAdded();
  }
}

/////////////////////////////////////////////////
void ModelEditorPalette::AddItem(QWidget *_item,
    const std::string &_category)
{
  std::string category = _category;
  if (category.empty())
    category = "Other";

  auto iter = this->dataPtr->categories.find(category);
  QGridLayout *catLayout = NULL;
  if (iter == this->dataPtr->categories.end())
  {
    catLayout = new QGridLayout();
    this->dataPtr->categories[category] = catLayout;

    std::string catStr =
        "<font size=4 color='white'>" + category + "</font>";
    QLabel *catLabel = new QLabel(tr(catStr.c_str()));
    this->dataPtr->otherItemsLayout->addWidget(catLabel);
    this->dataPtr->otherItemsLayout->addLayout(catLayout);
  }
  else
    catLayout = iter->second;

  int rowWidth = 3;
  int row = catLayout->count() / rowWidth;
  int col = catLayout->count() % rowWidth;
  catLayout->addWidget(_item, row, col);
}

/////////////////////////////////////////////////
void ModelEditorPalette::InsertWidget(unsigned int _index, QWidget *_widget)
{
  if (static_cast<int>(_index) > this->dataPtr->splitter->count())
  {
    gzerr << "Unable to add widget, index out of range " << std::endl;
    return;
  }

  // set equal size for now. There should always be at least one widget
  // (render3DFrame) in the splitter.
  GZ_ASSERT(this->dataPtr->splitter->count() > 0,
      "ModelEditorPalette splitter has no child widget");

  this->dataPtr->splitter->insertWidget(_index, _widget);
  this->dataPtr->splitter->setStretchFactor(_index, 1);
}

/////////////////////////////////////////////////
void ModelEditorPalette::RemoveWidget(QWidget *_widget)
{
  int idx = this->dataPtr->splitter->indexOf(_widget);
  if (idx > 0)
    this->dataPtr->splitter->widget(idx)->hide();
}

/////////////////////////////////////////////////
void ModelEditorPalette::CreateJoint(const std::string &_type)
{
  event::Events::setSelectedEntity("", "normal");
  this->dataPtr->modelCreator->AddJoint(_type);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnLinkAdded()
{
  this->dataPtr->linkButtonGroup->setExclusive(false);
  if (this->dataPtr->linkButtonGroup->checkedButton())
    this->dataPtr->linkButtonGroup->checkedButton()->setChecked(false);
  this->dataPtr->linkButtonGroup->setExclusive(true);
}

/////////////////////////////////////////////////
bool ModelEditorPalette::OnKeyPress(const common::KeyEvent &_event)
{
  if (_event.key == Qt::Key_Escape)
  {
    // call the slots to uncheck the buttons
    this->OnLinkAdded();
  }
  if (_event.key == Qt::Key_Delete)
  {
    event::Events::setSelectedEntity("", "normal");
    g_arrowAct->trigger();
  }
  return false;
}

/////////////////////////////////////////////////
gui::ModelCreator *ModelEditorPalette::ModelCreator()
{
  return this->dataPtr->modelCreator;
}
