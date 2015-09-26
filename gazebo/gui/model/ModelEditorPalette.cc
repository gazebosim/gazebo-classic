/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/model/ExtrudeDialog.hh"
#include "gazebo/gui/model/ImportDialog.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelEditorPalette.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelEditorPalette::ModelEditorPalette(QWidget *_parent)
    : QWidget(_parent)
{
  this->setObjectName("modelEditorPalette");

  this->modelDefaultName = "Untitled";

  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Simple Shapes
  QLabel *shapesLabel = new QLabel(tr(
       "<font size=4 color='white'>Simple Shapes</font>"));

  QHBoxLayout *shapesLayout = new QHBoxLayout;

  QSize toolButtonSize(70, 70);
  QSize iconSize(40, 40);

  // Cylinder button
  QToolButton *cylinderButton = new QToolButton(this);
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
  this->linkButtonGroup = new QButtonGroup;
  this->linkButtonGroup->addButton(cylinderButton);
  this->linkButtonGroup->addButton(sphereButton);
  this->linkButtonGroup->addButton(boxButton);
  this->linkButtonGroup->addButton(customButton);

  // Model Settings
  QLabel *settingsLabel = new QLabel(tr(
       "<font size=4 color='white'>Model Settings</font>"));

  QGridLayout *settingsLayout = new QGridLayout;

  // Model name
  QLabel *modelLabel = new QLabel(tr("Model Name: "));
  this->modelNameEdit = new QLineEdit();
  this->modelNameEdit->setText(tr(this->modelDefaultName.c_str()));
  connect(this->modelNameEdit, SIGNAL(textChanged(QString)), this,
      SLOT(OnNameChanged(QString)));

  // Static
  QLabel *staticLabel = new QLabel(tr("Static:"));
  this->staticCheck = new QCheckBox;
  this->staticCheck->setChecked(false);
  connect(this->staticCheck, SIGNAL(clicked()), this, SLOT(OnStatic()));

  // Auto disable
  QLabel *autoDisableLabel = new QLabel(tr("Auto-disable:"));
  this->autoDisableCheck = new QCheckBox;
  this->autoDisableCheck->setChecked(true);
  connect(this->autoDisableCheck, SIGNAL(clicked()), this,
      SLOT(OnAutoDisable()));

  settingsLayout->addWidget(modelLabel, 0, 0);
  settingsLayout->addWidget(this->modelNameEdit, 0, 1);
  settingsLayout->addWidget(staticLabel, 1, 0);
  settingsLayout->addWidget(this->staticCheck, 1, 1);
  settingsLayout->addWidget(autoDisableLabel, 2, 0);
  settingsLayout->addWidget(this->autoDisableCheck, 2, 1);

  this->modelCreator = new ModelCreator();
  connect(modelCreator, SIGNAL(LinkAdded()), this, SLOT(OnLinkAdded()));

  this->otherItemsLayout = new QVBoxLayout();
  this->otherItemsLayout->setContentsMargins(0, 0, 0, 0);

  // Palette layout
  QVBoxLayout *paletteLayout = new QVBoxLayout();
  paletteLayout->addWidget(shapesLabel);
  paletteLayout->addLayout(shapesLayout);
  paletteLayout->addWidget(customShapesLabel);
  paletteLayout->addLayout(customLayout);
  paletteLayout->addLayout(this->otherItemsLayout);
  paletteLayout->addItem(new QSpacerItem(30, 30, QSizePolicy::Minimum,
      QSizePolicy::Minimum));
  paletteLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
  QWidget *paletteWidget = new QWidget();
  paletteWidget->setLayout(paletteLayout);

  // Model tree
  this->modelTreeWidget = new QTreeWidget();
  this->modelTreeWidget->setObjectName("modelTreeWidget");
  this->modelTreeWidget->setColumnCount(1);
  this->modelTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->modelTreeWidget->header()->hide();
  this->modelTreeWidget->setFocusPolicy(Qt::NoFocus);
  this->modelTreeWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
  this->modelTreeWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
  this->modelTreeWidget->setVerticalScrollMode(
      QAbstractItemView::ScrollPerPixel);

  // Model Plugins
  this->modelPluginsItem = new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0),
      QStringList(QString("%1").arg(tr("Model Plugins"))));
  this->modelPluginsItem->setData(0, Qt::UserRole,
      QVariant(tr("Model Plugins")));
  QFont headerFont = this->modelPluginsItem->font(0);
  headerFont.setBold(true);
  headerFont.setPointSize(1.0 * headerFont.pointSize());
  this->modelPluginsItem->setFont(0, headerFont);
  this->modelTreeWidget->addTopLevelItem(this->modelPluginsItem);

  // Nested models
  this->nestedModelsItem = new QTreeWidgetItem(
      static_cast<QTreeWidgetItem *>(0),
      QStringList(QString("%1").arg(tr("Models"))));
  this->nestedModelsItem->setData(0, Qt::UserRole,
      QVariant(tr("Nested Models")));
  this->nestedModelsItem->setFont(0, headerFont);
  this->modelTreeWidget->addTopLevelItem(this->nestedModelsItem);

  // Links
  this->linksItem = new QTreeWidgetItem(
      static_cast<QTreeWidgetItem *>(0),
      QStringList(QString("%1").arg(tr("Links"))));
  this->linksItem->setData(0, Qt::UserRole, QVariant(tr("Links")));
  this->linksItem->setFont(0, headerFont);
  this->modelTreeWidget->addTopLevelItem(this->linksItem);

  // Joints
  this->jointsItem = new QTreeWidgetItem(
      static_cast<QTreeWidgetItem*>(0),
      QStringList(QString("%1").arg(tr("Joints"))));
  this->jointsItem->setData(0, Qt::UserRole, QVariant(tr("Joints")));
  this->jointsItem->setFont(0, headerFont);
  this->modelTreeWidget->addTopLevelItem(this->jointsItem);

  connect(this->modelTreeWidget,
      SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)),
      this, SLOT(OnItemDoubleClicked(QTreeWidgetItem *, int)));

  connect(this->modelTreeWidget,
      SIGNAL(itemClicked(QTreeWidgetItem *, int)),
      this, SLOT(OnItemClicked(QTreeWidgetItem *, int)));

  connect(this->modelTreeWidget, SIGNAL(itemSelectionChanged()),
      this, SLOT(OnItemSelectionChanged()));

  connect(this->modelTreeWidget,
      SIGNAL(customContextMenuRequested(const QPoint &)),
      this, SLOT(OnCustomContextMenu(const QPoint &)));

  // Model layout
  QVBoxLayout *modelLayout = new QVBoxLayout();
  modelLayout->addWidget(settingsLabel);
  modelLayout->addLayout(settingsLayout);
  modelLayout->addWidget(this->modelTreeWidget);
  modelLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
  QWidget *modelWidget = new QWidget();
  modelWidget->setLayout(modelLayout);

  // Main layout
  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;

  QSplitter *splitter = new QSplitter(Qt::Vertical, this);
  splitter->addWidget(paletteWidget);
  splitter->addWidget(modelWidget);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 2);
  splitter->setCollapsible(0, false);
  splitter->setCollapsible(1, false);

  frameLayout->addWidget(splitter);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);

  mainLayout->addWidget(frame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  KeyEventHandler::Instance()->AddPressFilter("model_editor",
    boost::bind(&ModelEditorPalette::OnKeyPress, this, _1));

  // Connections
  this->connections.push_back(
      gui::model::Events::ConnectSaveModel(
      boost::bind(&ModelEditorPalette::OnSaveModel, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectNewModel(
      boost::bind(&ModelEditorPalette::OnNewModel, this)));

  this->connections.push_back(
      gui::model::Events::ConnectModelPropertiesChanged(
      boost::bind(&ModelEditorPalette::OnModelPropertiesChanged, this, _1, _2,
      _3, _4)));

  this->connections.push_back(
      gui::model::Events::ConnectNestedModelInserted(
      boost::bind(&ModelEditorPalette::OnNestedModelInserted, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectLinkInserted(
      boost::bind(&ModelEditorPalette::OnLinkInserted, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectJointInserted(
      boost::bind(&ModelEditorPalette::OnJointInserted, this, _1, _2, _3, _4)));

  this->connections.push_back(
      gui::model::Events::ConnectModelPluginInserted(
      boost::bind(&ModelEditorPalette::OnModelPluginInserted, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectLinkRemoved(
      boost::bind(&ModelEditorPalette::OnLinkRemoved, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectJointRemoved(
      boost::bind(&ModelEditorPalette::OnJointRemoved, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectModelPluginRemoved(
      boost::bind(&ModelEditorPalette::OnModelPluginRemoved, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectJointNameChanged(
      boost::bind(&ModelEditorPalette::OnJointNameChanged, this, _1, _2)));

  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&ModelEditorPalette::OnSetSelectedEntity, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedLink(
       boost::bind(&ModelEditorPalette::OnSetSelectedLink, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedJoint(
       boost::bind(&ModelEditorPalette::OnSetSelectedJoint, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedModelPlugin(
     boost::bind(&ModelEditorPalette::OnSetSelectedModelPlugin, this, _1, _2)));
}

/////////////////////////////////////////////////
ModelEditorPalette::~ModelEditorPalette()
{
  delete this->modelCreator;
  this->modelCreator = NULL;
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnCylinder()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->modelCreator->AddLink(ModelCreator::LINK_CYLINDER);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSphere()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->modelCreator->AddLink(ModelCreator::LINK_SPHERE);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnBox()
{
  event::Events::setSelectedEntity("", "normal");
  g_arrowAct->trigger();

  this->modelCreator->AddLink(ModelCreator::LINK_BOX);
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
        this->modelCreator->AddShape(ModelCreator::LINK_MESH,
            math::Vector3::One, math::Pose::Zero, importDialog.GetImportPath());
      }
      else if (info.completeSuffix().toLower() == "svg")
      {
        ExtrudeDialog extrudeDialog(importDialog.GetImportPath(), this);
        extrudeDialog.deleteLater();
        if (extrudeDialog.exec() == QDialog::Accepted)
        {
          this->modelCreator->AddShape(ModelCreator::LINK_POLYLINE,
              math::Vector3(1.0/extrudeDialog.GetResolution(),
              1.0/extrudeDialog.GetResolution(),
              extrudeDialog.GetThickness()),
              math::Pose::Zero, importDialog.GetImportPath(),
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

  auto iter = this->categories.find(category);
  QGridLayout *catLayout = NULL;
  if (iter == this->categories.end())
  {
    catLayout = new QGridLayout();
    this->categories[category] = catLayout;

    std::string catStr =
        "<font size=4 color='white'>" + category + "</font>";
    QLabel *catLabel = new QLabel(tr(catStr.c_str()));
    this->otherItemsLayout->addWidget(catLabel);
    this->otherItemsLayout->addLayout(catLayout);
  }
  else
    catLayout = iter->second;

  int rowWidth = 3;
  int row = catLayout->count() / rowWidth;
  int col = catLayout->count() % rowWidth;
  catLayout->addWidget(_item, row, col);
}

/////////////////////////////////////////////////
void ModelEditorPalette::CreateJoint(const std::string &_type)
{
  event::Events::setSelectedEntity("", "normal");
  this->modelCreator->AddJoint(_type);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnLinkAdded()
{
  this->linkButtonGroup->setExclusive(false);
  if (this->linkButtonGroup->checkedButton())
    this->linkButtonGroup->checkedButton()->setChecked(false);
  this->linkButtonGroup->setExclusive(true);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnAutoDisable()
{
  this->modelCreator->SetAutoDisable(this->autoDisableCheck->isChecked());
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnStatic()
{
  this->modelCreator->SetStatic(this->staticCheck->isChecked());
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnModelPropertiesChanged(
    bool _static, bool _autoDisable, const math::Pose &/*_pose*/,
    const std::string &_name)
{
  this->staticCheck->setChecked(_static);
  this->autoDisableCheck->setChecked(_autoDisable);
  this->modelNameEdit->setText(tr(_name.c_str()));
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
void ModelEditorPalette::OnItemSelectionChanged()
{
  QList<QTreeWidgetItem *> items = this->modelTreeWidget->selectedItems();

  // update and signal new selection
  for (auto const item : items)
  {
    int idx = this->selected.indexOf(item);
    if (idx >= 0)
    {
      this->selected.removeAt(idx);
      continue;
    }
    std::string name = item->data(0, Qt::UserRole).toString().toStdString();
    std::string type = item->data(1, Qt::UserRole).toString().toStdString();

    if (type == "Link")
      gui::model::Events::setSelectedLink(name, true);
    else if (type == "Joint")
      gui::model::Events::setSelectedJoint(name, true);
    else if (type == "Model Plugin")
      gui::model::Events::setSelectedModelPlugin(name, true);
  }

  // deselect
  for (auto const item : this->selected)
  {
    if (item)
    {
      std::string name = item->data(0, Qt::UserRole).toString().toStdString();
      std::string type = item->data(1, Qt::UserRole).toString().toStdString();

      if (type == "Link")
        gui::model::Events::setSelectedLink(name, false);
      else if (type == "Joint")
        gui::model::Events::setSelectedJoint(name, false);
    else if (type == "Model Plugin")
      gui::model::Events::setSelectedModelPlugin(name, false);
    }
  }

  this->selected = items;
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSetSelectedEntity(const std::string &/*_name*/,
    const std::string &/*_mode*/)
{
  // deselect all
  for (auto &item : this->selected)
  {
    if (item)
      item->setSelected(false);
  }
}

/////////////////////////////////////////////////
ModelCreator *ModelEditorPalette::GetModelCreator()
{
  return this->modelCreator;
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnNameChanged(const QString &_name)
{
  gui::model::Events::modelNameChanged(_name.toStdString());
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnNewModel()
{
  this->modelNameEdit->setText(tr(this->modelDefaultName.c_str()));

  this->ClearModelTree();
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSaveModel(const std::string &_saveName)
{
  this->modelNameEdit->setText(tr(_saveName.c_str()));
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnItemDoubleClicked(QTreeWidgetItem *_item,
    int /*_column*/)
{
  if (_item)
  {
    std::string name = _item->data(0, Qt::UserRole).toString().toStdString();
    std::string type = _item->data(1, Qt::UserRole).toString().toStdString();

    if (type == "Link")
      gui::model::Events::openLinkInspector(name);
    else if (type == "Joint")
      gui::model::Events::openJointInspector(name);
    else if (type == "Model Plugin")
      gui::model::Events::openModelPluginInspector(name);
  }
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnItemClicked(QTreeWidgetItem *_item,
    int /*_column*/)
{
  if (_item)
  {
    if (this->selected.empty())
      return;

    QTreeWidgetItem *item = this->selected[0];
    std::string selectedType =
        item->data(1, Qt::UserRole).toString().toStdString();

    std::string type = _item->data(1, Qt::UserRole).toString().toStdString();

    if (type != selectedType)
      this->DeselectType(selectedType);
  }
}

/////////////////////////////////////////////////
void ModelEditorPalette::DeselectType(const std::string &_type)
{
  QObject::disconnect(this->modelTreeWidget, SIGNAL(itemSelectionChanged()),
      this, SLOT(OnItemSelectionChanged()));

  for (auto it = this->selected.begin(); it != this->selected.end();)
  {
    std::string name = (*it)->data(0, Qt::UserRole).toString().toStdString();
    std::string type = (*it)->data(1, Qt::UserRole).toString().toStdString();
    if (type == _type)
    {
      (*it)->setSelected(false);
      it = this->selected.erase(it);
      if (type == "Link")
        gui::model::Events::setSelectedLink(name, false);
      else if (type == "Joint")
        gui::model::Events::setSelectedJoint(name, false);
      else if (type == "Model Plugin")
        gui::model::Events::setSelectedModelPlugin(name, false);
    }
    else
      ++it;
  }

  this->selected = this->modelTreeWidget->selectedItems();

  QObject::connect(this->modelTreeWidget, SIGNAL(itemSelectionChanged()),
      this, SLOT(OnItemSelectionChanged()));
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnCustomContextMenu(const QPoint &_pt)
{
  QTreeWidgetItem *item = this->modelTreeWidget->itemAt(_pt);

  if (item)
  {
    std::string name = item->data(0, Qt::UserRole).toString().toStdString();
    std::string type = item->data(1, Qt::UserRole).toString().toStdString();

    if (type == "Link")
      gui::model::Events::showLinkContextMenu(name);
    else if (type == "Joint")
      gui::model::Events::showJointContextMenu(name);
    else if (type == "Model Plugin")
      gui::model::Events::showModelPluginContextMenu(name);
  }
}

/////////////////////////////////////////////////
QTreeWidgetItem *ModelEditorPalette::FindItemByData(const std::string &_data,
    const QTreeWidgetItem &_parentItem)
{
  for (int i = 0; i < _parentItem.childCount(); ++i)
  {
    QTreeWidgetItem *item = _parentItem.child(i);
    std::string itemId =
        item->data(0, Qt::UserRole).toString().toStdString();

    if (itemId == _data)
    {
      return item;
    }

    item = this->FindItemByData(_data, *item);
    if (item)
      return item;
  }
  return NULL;
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnNestedModelInserted(
    const std::string &_nestedModelName)
{
  // Divide the name into parent scoped name and leaf name
  std::string parentScopedName = _nestedModelName;
  std::string leafName = _nestedModelName;
  size_t idx = _nestedModelName.rfind("::");
  if (idx != std::string::npos)
  {
    parentScopedName = _nestedModelName.substr(0, idx);
    leafName = _nestedModelName.substr(idx+2);
  }

  // Top level by default
  QTreeWidgetItem *parentItem = this->nestedModelsItem;

  // If the parent model is a nested model
  QTreeWidgetItem *item = this->FindItemByData(parentScopedName,
      *this->nestedModelsItem);
  if (item)
    parentItem = item;

  // check if nested model already exists
  auto treeItems = this->modelTreeWidget->findItems(tr(leafName.c_str()),
      Qt::MatchExactly | Qt::MatchRecursive);
  if (!treeItems.empty())
    return;

  QTreeWidgetItem *newNestedModelItem =
      new QTreeWidgetItem(parentItem,
      QStringList(QString("%1").arg(QString::fromStdString(leafName))));

  newNestedModelItem->setData(0, Qt::UserRole, _nestedModelName.c_str());
  newNestedModelItem->setData(1, Qt::UserRole, "Nested Model");

  this->nestedModelsItem->setExpanded(true);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnLinkInserted(const std::string &_linkName)
{
  std::string leafName = _linkName;
  size_t idx = _linkName.rfind("::");
  if (idx != std::string::npos)
    leafName = _linkName.substr(idx+2);

  QTreeWidgetItem *newLinkItem = new QTreeWidgetItem(this->linksItem,
      QStringList(QString("%1").arg(QString::fromStdString(leafName))));

  newLinkItem->setData(0, Qt::UserRole, _linkName.c_str());
  newLinkItem->setData(1, Qt::UserRole, "Link");

  this->linksItem->setExpanded(true);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnJointInserted(const std::string &_jointId,
    const std::string &_jointName, const std::string &/*_parentName*/,
    const std::string &/*_childName*/)
{
  std::string leafName = _jointName;
  size_t idx = _jointName.rfind("::");
  if (idx != std::string::npos)
    leafName = _jointName.substr(idx+2);

  QTreeWidgetItem *newJointItem = new QTreeWidgetItem(this->jointsItem,
      QStringList(QString("%1").arg(QString::fromStdString(leafName))));

  newJointItem->setData(0, Qt::UserRole, _jointId.c_str());
  newJointItem->setData(1, Qt::UserRole, "Joint");

  this->jointsItem->setExpanded(true);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnModelPluginInserted(
    const std::string &_modelPluginName)
{
  QTreeWidgetItem *newModelPluginItem = new QTreeWidgetItem(
      this->modelPluginsItem, QStringList(QString("%1").arg(
      QString::fromStdString(_modelPluginName))));

  newModelPluginItem->setData(0, Qt::UserRole, _modelPluginName.c_str());
  newModelPluginItem->setData(1, Qt::UserRole, "Model Plugin");

  this->modelPluginsItem->setExpanded(true);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnLinkRemoved(const std::string &_linkId)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);

  QTreeWidgetItem *item = this->FindItemByData(_linkId, *this->linksItem);
  if (item)
    this->linksItem->takeChild(this->linksItem->indexOfChild(item));
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnJointRemoved(const std::string &_jointId)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);

  QTreeWidgetItem *item = this->FindItemByData(_jointId, *this->jointsItem);
  if (item)
    this->jointsItem->takeChild(this->jointsItem->indexOfChild(item));
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnModelPluginRemoved(const std::string &_pluginId)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  for (int i = 0; i < this->modelPluginsItem->childCount(); ++i)
  {
    QTreeWidgetItem *item = this->modelPluginsItem->child(i);
    if (!item)
      continue;
    std::string listData = item->data(0, Qt::UserRole).toString().toStdString();

    if (listData == _pluginId)
    {
      this->modelPluginsItem->takeChild(this->modelPluginsItem->indexOfChild(
          item));
      break;
    }
  }
}

/////////////////////////////////////////////////
void ModelEditorPalette::ClearModelTree()
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  // Remove all nested models
  this->nestedModelsItem->takeChildren();
  // Remove all links
  this->linksItem->takeChildren();
  // Remove all joints
  this->jointsItem->takeChildren();
  // Remove all model plugins
  this->modelPluginsItem->takeChildren();
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnJointNameChanged(const std::string &_jointId,
    const std::string &_newJointName)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);

  QTreeWidgetItem *item = this->FindItemByData(_jointId, *this->jointsItem);
  if (item)
    item->setText(0, QString::fromStdString(_newJointName));
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSetSelectedLink(const std::string &_linkId,
    bool _selected)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);

  QTreeWidgetItem *item = this->FindItemByData(_linkId, *this->linksItem);
  if (item)
    item->setSelected(_selected);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSetSelectedJoint(const std::string &_jointId,
    bool _selected)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);

  QTreeWidgetItem *item = this->FindItemByData(_jointId, *this->jointsItem);
  if (item)
    item->setSelected(_selected);
}

/////////////////////////////////////////////////
void ModelEditorPalette::OnSetSelectedModelPlugin(const std::string &_name,
    bool _selected)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  for (int i = 0; i < this->modelPluginsItem->childCount(); ++i)
  {
    QTreeWidgetItem *item = this->modelPluginsItem->child(i);
    if (!item)
      continue;
    std::string listData = item->data(0, Qt::UserRole).toString().toStdString();

    if (listData == _name)
    {
      item->setSelected(_selected);
      break;
    }
  }
}
