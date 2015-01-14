/*
 * Copyright 2015 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/PartVisualConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PartVisualConfig::PartVisualConfig()
{
  this->setObjectName("PartVisualConfig");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->visualsTreeWidget = new QTreeWidget();
  this->visualsTreeWidget->setColumnCount(1);
  this->visualsTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->visualsTreeWidget->header()->hide();
  this->visualsTreeWidget->setIndentation(4);

  this->visualsTreeWidget->setSelectionMode(QAbstractItemView::NoSelection);
  connect(this->visualsTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
      this, SLOT(OnItemSelection(QTreeWidgetItem *, int)));

  QPushButton *addVisualButton = new QPushButton(tr("+ &Another Visual"));
  connect(addVisualButton, SIGNAL(clicked()), this, SLOT(OnAddVisual()));

  mainLayout->addWidget(this->visualsTreeWidget);
  mainLayout->addWidget(addVisualButton);
  this->setLayout(mainLayout);

  this->counter = 0;
  this->signalMapper = new QSignalMapper(this);

  connect(this->signalMapper, SIGNAL(mapped(int)),
     this, SLOT(OnRemoveVisual(int)));
}

/////////////////////////////////////////////////
PartVisualConfig::~PartVisualConfig()
{
}

/////////////////////////////////////////////////
void PartVisualConfig::OnAddVisual()
{
  std::stringstream visualIndex;
  visualIndex << "visual_" << this->counter;
  this->AddVisual(visualIndex.str());
  emit VisualAdded(visualIndex.str());
}

/////////////////////////////////////////////////
unsigned int PartVisualConfig::GetVisualCount() const
{
  return this->configs.size();
}

/////////////////////////////////////////////////
void PartVisualConfig::Reset()
{
  std::map<int, VisualConfigData *>::iterator it;
  for (it = this->configs.begin(); it != this->configs.end(); ++it)
    delete it->second;

  this->configs.clear();
  this->visualsTreeWidget->clear();
}

/////////////////////////////////////////////////
void PartVisualConfig::AddVisual(const std::string &_name,
    const msgs::Visual *_visualMsg)
{
  // Create a top-level tree item for the path
  QTreeWidgetItem *visualItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0));
  this->visualsTreeWidget->addTopLevelItem(visualItem);

  QWidget *visualItemWidget = new QWidget;
  QHBoxLayout *visualItemLayout = new QHBoxLayout;
  QLabel *visualLabel = new QLabel(QString(_name.c_str()));

  QPushButton *removeVisualButton = new QPushButton(tr("Remove"));
  connect(removeVisualButton, SIGNAL(clicked()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(removeVisualButton, this->counter);

  visualItemLayout->addWidget(visualLabel);
  visualItemLayout->addWidget(removeVisualButton);
  visualItemLayout->setContentsMargins(10, 0, 0, 0);
  visualItemWidget->setLayout(visualItemLayout);
  this->visualsTreeWidget->setItemWidget(visualItem, 0, visualItemWidget);

  QTreeWidgetItem *visualChildItem =
    new QTreeWidgetItem(visualItem);

  QWidget *visualWidget = new QWidget;
  QVBoxLayout *visualLayout = new QVBoxLayout;

  ConfigWidget *configWidget = new ConfigWidget;

  if (_visualMsg)
  {
    configWidget->Load(_visualMsg);
  }
  else
  {
    msgs::Visual visualMsg;
    configWidget->Load(&visualMsg);
  }

  configWidget->SetWidgetVisible("id", false);
  configWidget->SetWidgetVisible("name", false);
  configWidget->SetWidgetVisible("parent_name", false);
  configWidget->SetWidgetVisible("parent_id", false);
  configWidget->SetWidgetVisible("delete_me", false);
  configWidget->SetWidgetVisible("is_static", false);
  configWidget->SetWidgetVisible("visible", false);
  configWidget->SetWidgetVisible("scale", false);
  configWidget->SetWidgetVisible("plugin", false);
  configWidget->SetWidgetReadOnly("id", true);
  configWidget->SetWidgetReadOnly("name", true);
  configWidget->SetWidgetReadOnly("parent_name", true);
  configWidget->SetWidgetReadOnly("parent_id", true);
  configWidget->SetWidgetReadOnly("delete_me", true);
  configWidget->SetWidgetReadOnly("is_static", true);
  configWidget->SetWidgetReadOnly("visible", true);
  configWidget->SetWidgetReadOnly("scale", true);
  configWidget->SetWidgetReadOnly("plugin", true);

  VisualConfigData *configData = new VisualConfigData;
  configData->configWidget = configWidget;
  configData->id =  this->counter;
  configData->treeItem = visualItem;
  configData->name = _name;
  this->configs[this->counter] = configData;
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(configWidget);
  visualLayout->addWidget(scrollArea);
  this->counter++;

  visualLayout->setContentsMargins(0, 0, 0, 0);
  visualWidget->setLayout(visualLayout);
  visualWidget->setMinimumHeight(650);

  this->visualsTreeWidget->setItemWidget(visualChildItem, 0, visualWidget);
  visualItem->setExpanded(false);
  visualChildItem->setExpanded(false);
}

/////////////////////////////////////////////////
void PartVisualConfig::UpdateVisual(const std::string &_name,
    const msgs::Visual *_visualMsg)
{
  std::map<int, VisualConfigData *>::iterator it;

  for (it = this->configs.begin(); it != this->configs.end(); ++it)
  {
    if (it->second->name == _name)
    {
      VisualConfigData *configData = it->second;
      configData->configWidget->UpdateFromMsg(_visualMsg);
      break;
    }
  }
}

/////////////////////////////////////////////////
void PartVisualConfig::OnItemSelection(QTreeWidgetItem *_item,
                                         int /*_column*/)
{
  if (_item && _item->childCount() > 0)
    _item->setExpanded(!_item->isExpanded());
}

/////////////////////////////////////////////////
void PartVisualConfig::OnRemoveVisual(int _id)
{
  std::map<int, VisualConfigData *>::iterator it = this->configs.find(_id);
  if (it == this->configs.end())
  {
    gzerr << "Visual not found " << std::endl;
    return;
  }

  VisualConfigData *configData = this->configs[_id];

  int index = this->visualsTreeWidget->indexOfTopLevelItem(
      configData->treeItem);
  this->visualsTreeWidget->takeTopLevelItem(index);

  emit VisualRemoved(this->configs[_id]->name);
  this->configs.erase(it);
}

/////////////////////////////////////////////////
msgs::Visual *PartVisualConfig::GetData(const std::string &_name) const
{
  std::map<int, VisualConfigData *>::const_iterator it;
  for (it = this->configs.begin(); it != this->configs.end(); ++it)
  {
    if (it->second->name == _name)
      return dynamic_cast<msgs::Visual *>(it->second->configWidget->GetMsg());
  }
  return NULL;
}

/////////////////////////////////////////////////
void PartVisualConfig::SetGeometrySize(const std::string &_name,
    const math::Vector3 &_size)
{
  std::map<int, VisualConfigData *>::iterator it;
  for (it = this->configs.begin(); it != this->configs.end(); ++it)
  {
    if (it->second->name == _name)
    {
      math::Vector3 dimensions;
      std::string type = it->second->configWidget->GetGeometryWidgetValue(
          "geometry", dimensions);
      it->second->configWidget->SetGeometryWidgetValue("geometry", type,
          _size);
      break;
    }
  }
}
