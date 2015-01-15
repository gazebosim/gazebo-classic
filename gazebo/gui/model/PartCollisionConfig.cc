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
#include "gazebo/gui/model/PartCollisionConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PartCollisionConfig::PartCollisionConfig()
{
  this->setObjectName("PartCollisionConfig");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->collisionsTreeWidget = new QTreeWidget();
  this->collisionsTreeWidget->setColumnCount(1);
  this->collisionsTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->collisionsTreeWidget->header()->hide();
  this->collisionsTreeWidget->setIndentation(4);

  this->collisionsTreeWidget->setSelectionMode(QAbstractItemView::NoSelection);
  connect(this->collisionsTreeWidget,
      SIGNAL(itemClicked(QTreeWidgetItem *, int)),
      this, SLOT(OnItemSelection(QTreeWidgetItem *, int)));

  QPushButton *addCollisionButton = new QPushButton(tr("+ &Another Collision"));
  connect(addCollisionButton, SIGNAL(clicked()), this, SLOT(OnAddCollision()));

  mainLayout->addWidget(this->collisionsTreeWidget);
  mainLayout->addWidget(addCollisionButton);
  this->setLayout(mainLayout);

  this->counter = 0;
  this->signalMapper = new QSignalMapper(this);

  connect(this->signalMapper, SIGNAL(mapped(int)),
     this, SLOT(OnRemoveCollision(int)));
}

/////////////////////////////////////////////////
PartCollisionConfig::~PartCollisionConfig()
{
}

/////////////////////////////////////////////////
void PartCollisionConfig::OnAddCollision()
{
  std::stringstream collisionIndex;
  collisionIndex << "collision_" << this->counter;
  this->AddCollision(collisionIndex.str());
  emit CollisionAdded(collisionIndex.str());
}

/////////////////////////////////////////////////
unsigned int PartCollisionConfig::GetCollisionCount() const
{
  return this->configs.size();
}

/////////////////////////////////////////////////
void PartCollisionConfig::Reset()
{
  std::map<int, CollisionConfigData *>::iterator it;
  for (it = this->configs.begin(); it != this->configs.end(); ++it)
    delete it->second;

  this->configs.clear();
  this->collisionsTreeWidget->clear();
}

/////////////////////////////////////////////////
void PartCollisionConfig::UpdateCollision(const std::string &_name,
    const msgs::Collision *_collisionMsg)
{
  std::map<int, CollisionConfigData *>::iterator it;

  for (it = this->configs.begin(); it != this->configs.end(); ++it)
  {
    if (it->second->name == _name)
    {
      CollisionConfigData *configData = it->second;
      configData->configWidget->UpdateFromMsg(_collisionMsg);
      break;
    }
  }
}

/////////////////////////////////////////////////
void PartCollisionConfig::AddCollision(const std::string &_name,
    const msgs::Collision *_collisionMsg)
{
  // Create a top-level tree item for the path
  QTreeWidgetItem *collisionItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0));
  this->collisionsTreeWidget->addTopLevelItem(collisionItem);

  QWidget *collisionItemWidget = new QWidget;
  QHBoxLayout *collisionItemLayout = new QHBoxLayout;
  QLabel *collisionLabel = new QLabel(QString(_name.c_str()));

  QPushButton *removeCollisionButton = new QPushButton(tr("Remove"));
  connect(removeCollisionButton, SIGNAL(clicked()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(removeCollisionButton, this->counter);

  collisionItemLayout->addWidget(collisionLabel);
  collisionItemLayout->addWidget(removeCollisionButton);
  collisionItemLayout->setContentsMargins(10, 0, 0, 0);
  collisionItemWidget->setLayout(collisionItemLayout);
  this->collisionsTreeWidget->setItemWidget(collisionItem, 0,
      collisionItemWidget);

  QTreeWidgetItem *collisionChildItem =
    new QTreeWidgetItem(collisionItem);

  QWidget *collisionWidget = new QWidget;
  QVBoxLayout *collisionLayout = new QVBoxLayout;

  ConfigWidget *configWidget = new ConfigWidget;

  msgs::Collision msgToLoad;
  if (_collisionMsg)
    msgToLoad = *_collisionMsg;

  // set default values
  // TODO: auto-fill them with SDF defaults
  if (!msgToLoad.has_max_contacts())
    msgToLoad.set_max_contacts(10);
  msgs::Surface *surfaceMsg = msgToLoad.mutable_surface();
  if (!surfaceMsg->has_bounce_threshold())
    surfaceMsg->set_bounce_threshold(10e5);
  if (!surfaceMsg->has_soft_erp())
      surfaceMsg->set_soft_erp(0.2);
  if (!surfaceMsg->has_kp())
    surfaceMsg->set_kp(10e12);
  if (!surfaceMsg->has_kd())
    surfaceMsg->set_kd(1.0);
  if (!surfaceMsg->has_max_vel())
    surfaceMsg->set_max_vel(0.01);
  if (!surfaceMsg->has_collide_without_contact_bitmask())
    surfaceMsg->set_collide_without_contact_bitmask(1);
  msgs::Friction *frictionMsg = surfaceMsg->mutable_friction();
  if (!frictionMsg->has_mu())
    frictionMsg->set_mu(1.0);
  if (!frictionMsg->has_mu2())
    frictionMsg->set_mu2(1.0);

  configWidget->Load(&msgToLoad);

  configWidget->SetWidgetVisible("id", false);
  configWidget->SetWidgetVisible("name", false);
  configWidget->SetWidgetReadOnly("id", true);
  configWidget->SetWidgetReadOnly("name", true);

  CollisionConfigData *configData = new CollisionConfigData;
  configData->configWidget = configWidget;
  configData->id =  this->counter;
  configData->treeItem = collisionItem;
  configData->name = _name;
  this->configs[this->counter] = configData;
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(configWidget);
  collisionLayout->addWidget(scrollArea);
  this->counter++;

  collisionLayout->setContentsMargins(0, 0, 0, 0);
  collisionWidget->setLayout(collisionLayout);
  collisionWidget->setMinimumHeight(650);

  this->collisionsTreeWidget->setItemWidget(collisionChildItem, 0,
      collisionWidget);
  collisionItem->setExpanded(false);
  collisionChildItem->setExpanded(false);
}

/////////////////////////////////////////////////
void PartCollisionConfig::OnItemSelection(QTreeWidgetItem *_item,
                                         int /*_column*/)
{
  if (_item && _item->childCount() > 0)
    _item->setExpanded(!_item->isExpanded());
}


/////////////////////////////////////////////////
void PartCollisionConfig::OnRemoveCollision(int _id)
{
  std::map<int, CollisionConfigData *>::iterator it = this->configs.find(_id);
  if (it == this->configs.end())
  {
    gzerr << "Collision not found " << std::endl;
    return;
  }

  CollisionConfigData *configData = this->configs[_id];

  int index = this->collisionsTreeWidget->indexOfTopLevelItem(
      configData->treeItem);
  this->collisionsTreeWidget->takeTopLevelItem(index);

  emit CollisionRemoved(this->configs[_id]->name);
  this->configs.erase(it);
}

/////////////////////////////////////////////////
msgs::Collision *PartCollisionConfig::GetData(const std::string &_name) const
{
  std::map<int, CollisionConfigData *>::const_iterator it;
  for (it = this->configs.begin(); it != this->configs.end(); ++it)
  {
    std::string name = it->second->name;
    if (name == _name)
    {
      return dynamic_cast<msgs::Collision *>(
          it->second->configWidget->GetMsg());
    }
  }
  return NULL;
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetGeometrySize(const std::string &_name,
    const math::Vector3 &_size)
{
  std::map<int, CollisionConfigData *>::iterator it;
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
