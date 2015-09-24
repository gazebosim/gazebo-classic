/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/common/Events.hh"

#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/ModelTreeWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelTreeWidget::ModelTreeWidget(QWidget *_parent)
    : QWidget(_parent)
{
  this->setObjectName("ModelTreeWidget");

  this->modelDefaultName = "Untitled";

  QVBoxLayout *mainLayout = new QVBoxLayout;

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

//  this->modelCreator = new ModelCreator();
//  connect(modelCreator, SIGNAL(LinkAdded()), this, SLOT(OnLinkAdded()));

//  this->otherItemsLayout = new QVBoxLayout();
//  this->otherItemsLayout->setContentsMargins(0, 0, 0, 0);

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

  frameLayout->addWidget(modelWidget);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);

  mainLayout->addWidget(frame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  // Connections
  this->connections.push_back(
      gui::model::Events::ConnectSaveModel(
      boost::bind(&ModelTreeWidget::OnSaveModel, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectNewModel(
      boost::bind(&ModelTreeWidget::OnNewModel, this)));

  this->connections.push_back(
      gui::model::Events::ConnectModelPropertiesChanged(
      boost::bind(&ModelTreeWidget::OnModelPropertiesChanged, this, _1, _2,
      _3, _4)));

  this->connections.push_back(
      gui::model::Events::ConnectLinkInserted(
      boost::bind(&ModelTreeWidget::OnLinkInserted, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectJointInserted(
      boost::bind(&ModelTreeWidget::OnJointInserted, this, _1, _2, _3, _4)));

  this->connections.push_back(
      gui::model::Events::ConnectModelPluginInserted(
      boost::bind(&ModelTreeWidget::OnModelPluginInserted, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectLinkRemoved(
      boost::bind(&ModelTreeWidget::OnLinkRemoved, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectJointRemoved(
      boost::bind(&ModelTreeWidget::OnJointRemoved, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectModelPluginRemoved(
      boost::bind(&ModelTreeWidget::OnModelPluginRemoved, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectJointNameChanged(
      boost::bind(&ModelTreeWidget::OnJointNameChanged, this, _1, _2)));

  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&ModelTreeWidget::OnSetSelectedEntity, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedLink(
       boost::bind(&ModelTreeWidget::OnSetSelectedLink, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedJoint(
       boost::bind(&ModelTreeWidget::OnSetSelectedJoint, this, _1, _2)));

  this->connections.push_back(
     gui::model::Events::ConnectSetSelectedModelPlugin(
     boost::bind(&ModelTreeWidget::OnSetSelectedModelPlugin, this, _1, _2)));
}

/////////////////////////////////////////////////
ModelTreeWidget::~ModelTreeWidget()
{
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnAutoDisable()
{
//  this->modelCreator->SetAutoDisable(this->autoDisableCheck->isChecked());
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnStatic()
{
//  this->modelCreator->SetStatic(this->staticCheck->isChecked());
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnModelPropertiesChanged(
    bool _static, bool _autoDisable, const math::Pose &/*_pose*/,
    const std::string &_name)
{
  this->staticCheck->setChecked(_static);
  this->autoDisableCheck->setChecked(_autoDisable);
  this->modelNameEdit->setText(tr(_name.c_str()));
}

/*/////////////////////////////////////////////////
bool ModelTreeWidget::OnKeyPress(const common::KeyEvent &_event)
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
}*/

/////////////////////////////////////////////////
void ModelTreeWidget::OnItemSelectionChanged()
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
void ModelTreeWidget::OnSetSelectedEntity(const std::string &/*_name*/,
    const std::string &/*_mode*/)
{
  // deselect all
  for (auto &item : this->selected)
  {
    if (item)
      item->setSelected(false);
  }
}

/*/////////////////////////////////////////////////
ModelCreator *ModelTreeWidget::GetModelCreator()
{
  return this->modelCreator;
}*/

/////////////////////////////////////////////////
void ModelTreeWidget::OnNameChanged(const QString &_name)
{
  gui::model::Events::modelNameChanged(_name.toStdString());
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnNewModel()
{
  this->modelNameEdit->setText(tr(this->modelDefaultName.c_str()));

  this->ClearModelTree();
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnSaveModel(const std::string &_saveName)
{
  this->modelNameEdit->setText(tr(_saveName.c_str()));
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnItemDoubleClicked(QTreeWidgetItem *_item,
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
void ModelTreeWidget::OnItemClicked(QTreeWidgetItem *_item,
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
void ModelTreeWidget::DeselectType(const std::string &_type)
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
void ModelTreeWidget::OnCustomContextMenu(const QPoint &_pt)
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
void ModelTreeWidget::OnLinkInserted(const std::string &_linkName)
{
  std::string leafName = _linkName;
  size_t idx = _linkName.find_last_of("::");
  if (idx != std::string::npos)
    leafName = _linkName.substr(idx+1);

  QTreeWidgetItem *newLinkItem = new QTreeWidgetItem(this->linksItem,
      QStringList(QString("%1").arg(QString::fromStdString(leafName))));

  newLinkItem->setData(0, Qt::UserRole, _linkName.c_str());
  newLinkItem->setData(1, Qt::UserRole, "Link");
  this->modelTreeWidget->addTopLevelItem(newLinkItem);

  this->linksItem->setExpanded(true);
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnJointInserted(const std::string &_jointId,
    const std::string &_jointName, const std::string &/*_parentName*/,
    const std::string &/*_childName*/)
{
  std::string leafName = _jointName;
  size_t idx = _jointName.find_last_of("::");
  if (idx != std::string::npos)
    leafName = _jointName.substr(idx+1);

  QTreeWidgetItem *newJointItem = new QTreeWidgetItem(this->jointsItem,
      QStringList(QString("%1").arg(QString::fromStdString(leafName))));

  newJointItem->setData(0, Qt::UserRole, _jointId.c_str());
  newJointItem->setData(1, Qt::UserRole, "Joint");
  this->modelTreeWidget->addTopLevelItem(newJointItem);

  this->jointsItem->setExpanded(true);
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnModelPluginInserted(
    const std::string &_modelPluginName)
{
  QTreeWidgetItem *newModelPluginItem = new QTreeWidgetItem(
      this->modelPluginsItem, QStringList(QString("%1").arg(
      QString::fromStdString(_modelPluginName))));

  newModelPluginItem->setData(0, Qt::UserRole, _modelPluginName.c_str());
  newModelPluginItem->setData(1, Qt::UserRole, "Model Plugin");
  this->modelTreeWidget->addTopLevelItem(newModelPluginItem);

  this->modelPluginsItem->setExpanded(true);
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnLinkRemoved(const std::string &_linkId)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  for (int i = 0; i < this->linksItem->childCount(); ++i)
  {
    QTreeWidgetItem *item = this->linksItem->child(i);
    if (!item)
      continue;
    std::string listData = item->data(0, Qt::UserRole).toString().toStdString();

    if (listData == _linkId)
    {
      this->linksItem->takeChild(this->linksItem->indexOfChild(item));
      break;
    }
  }
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnJointRemoved(const std::string &_jointId)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  for (int i = 0; i < this->jointsItem->childCount(); ++i)
  {
    QTreeWidgetItem *item = this->jointsItem->child(i);
    if (!item)
      continue;
    std::string listData = item->data(0, Qt::UserRole).toString().toStdString();

    if (listData == _jointId)
    {
      this->jointsItem->takeChild(this->jointsItem->indexOfChild(item));
      break;
    }
  }
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnModelPluginRemoved(const std::string &_pluginId)
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
void ModelTreeWidget::ClearModelTree()
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  // Remove all links
  this->linksItem->takeChildren();
  // Remove all joints
  this->jointsItem->takeChildren();
  // Remove all model plugins
  this->modelPluginsItem->takeChildren();
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnJointNameChanged(const std::string &_jointId,
    const std::string &_newJointName)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  for (int i = 0; i < this->jointsItem->childCount(); ++i)
  {
    QTreeWidgetItem *item = this->jointsItem->child(i);
    if (!item)
      continue;
    std::string listData = item->data(0, Qt::UserRole).toString().toStdString();

    if (listData == _jointId)
    {
      item->setText(0, QString::fromStdString(_newJointName));
      break;
    }
  }
}

/////////////////////////////////////////////////
void ModelTreeWidget::OnSetSelectedLink(const std::string &_name,
    bool _selected)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  for (int i = 0; i < this->linksItem->childCount(); ++i)
  {
    QTreeWidgetItem *item = this->linksItem->child(i);
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

/////////////////////////////////////////////////
void ModelTreeWidget::OnSetSelectedJoint(const std::string &_name,
    bool _selected)
{
  std::unique_lock<std::recursive_mutex> lock(this->updateMutex);
  for (int i = 0; i < this->jointsItem->childCount(); ++i)
  {
    QTreeWidgetItem *item = this->jointsItem->child(i);
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

/////////////////////////////////////////////////
void ModelTreeWidget::OnSetSelectedModelPlugin(const std::string &_name,
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
