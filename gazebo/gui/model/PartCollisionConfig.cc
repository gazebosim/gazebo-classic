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

#include <iostream>

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

  if (_collisionMsg)
  {
    configWidget->Load(_collisionMsg);
  }
  else
  {
    msgs::Collision collisionMsg;
    configWidget->Load(&collisionMsg);
  }

  configWidget->SetWidgetVisible("id", false);
  configWidget->SetWidgetVisible("name", false);

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
  collisionLayout->setAlignment(Qt::AlignTop);
  collisionWidget->setLayout(collisionLayout);
  collisionWidget->setMinimumHeight(650);
  //collisionWidget->setMinimumWidth(600);

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

/*
/////////////////////////////////////////////////
void PartCollisionConfig::OnAddCollision()
{
  // Create a top-level tree item for the path
  std::stringstream collisionIndex;
  collisionIndex << "Collision " << counter;

  QTreeWidgetItem *collisionItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0));
  this->collisionsTreeWidget->addTopLevelItem(collisionItem);

  this->collisionItems[counter] = collisionItem;

  QWidget *collisionItemWidget = new QWidget;
  QHBoxLayout *collisionItemLayout = new QHBoxLayout;
  QLabel *collisionLabel = new QLabel(QString(collisionIndex.str().c_str()));

  QPushButton *removeCollisionButton = new QPushButton(tr("Remove"));
  connect(removeCollisionButton, SIGNAL(clicked()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(removeCollisionButton, counter);

  CollisionDataWidget *dataWidget = new CollisionDataWidget;
  dataWidget->id = counter;
  this->dataWidgets.push_back(dataWidget);
  counter++;

  collisionItemLayout->addWidget(collisionLabel);
  collisionItemLayout->addWidget(removeCollisionButton);
  collisionItemWidget->setLayout(collisionItemLayout);
  this->collisionsTreeWidget->setItemWidget(collisionItem, 0, collisionItemWidget);

  QTreeWidgetItem *collisionChildItem =
    new QTreeWidgetItem(collisionItem);

  QWidget *collisionWidget = new QWidget;
  QVBoxLayout *collisionLayout = new QVBoxLayout;

  QLabel *nameLabel = new QLabel(tr("Name:"));
  dataWidget->collisionNameLabel = new QLabel(tr(""));

  QLabel *geometryLabel = new QLabel(tr("Geometry:"));
  dataWidget->geometryComboBox = new QComboBox;
  dataWidget->geometryComboBox->addItem(tr("box"));
  dataWidget->geometryComboBox->addItem(tr("cylinder"));
  dataWidget->geometryComboBox->addItem(tr("sphere"));
  // geometryComboBox->addItem(tr("custom"));

  connect(dataWidget->geometryComboBox,
      SIGNAL(currentIndexChanged(const QString)),
      dataWidget, SLOT(GeometryChanged(const QString)));

  dataWidget->geomSizeXSpinBox = new QDoubleSpinBox;
  dataWidget->geomSizeXSpinBox->setRange(-1000, 1000);
  dataWidget->geomSizeXSpinBox->setSingleStep(0.01);
  dataWidget->geomSizeXSpinBox->setDecimals(3);
  dataWidget->geomSizeXSpinBox->setValue(1.000);

  dataWidget->geomSizeYSpinBox = new QDoubleSpinBox;
  dataWidget->geomSizeYSpinBox->setRange(-1000, 1000);
  dataWidget->geomSizeYSpinBox->setSingleStep(0.01);
  dataWidget->geomSizeYSpinBox->setDecimals(3);
  dataWidget->geomSizeYSpinBox->setValue(1.000);

  dataWidget->geomSizeZSpinBox = new QDoubleSpinBox;
  dataWidget->geomSizeZSpinBox->setRange(-1000, 1000);
  dataWidget->geomSizeZSpinBox->setSingleStep(0.01);
  dataWidget->geomSizeZSpinBox->setDecimals(3);
  dataWidget->geomSizeZSpinBox->setValue(1.000);

  QLabel *geomSizeXLabel = new QLabel(tr("x: "));
  QLabel *geomSizeYLabel = new QLabel(tr("y: "));
  QLabel *geomSizeZLabel = new QLabel(tr("z: "));

  QHBoxLayout *geomSizeLayout = new QHBoxLayout;
  geomSizeLayout->addWidget(geomSizeXLabel);
  geomSizeLayout->addWidget(dataWidget->geomSizeXSpinBox);
  geomSizeLayout->addWidget(geomSizeYLabel);
  geomSizeLayout->addWidget(dataWidget->geomSizeYSpinBox);
  geomSizeLayout->addWidget(geomSizeZLabel);
  geomSizeLayout->addWidget(dataWidget->geomSizeZSpinBox);

  QLabel *geomRadiusLabel = new QLabel(tr("radius: "));
  dataWidget->geomLengthLabel = new QLabel(tr("length: "));

  dataWidget->geomRadiusSpinBox = new QDoubleSpinBox;
  dataWidget->geomRadiusSpinBox->setRange(-1000, 1000);
  dataWidget->geomRadiusSpinBox->setSingleStep(0.01);
  dataWidget->geomRadiusSpinBox->setDecimals(3);
  dataWidget->geomRadiusSpinBox->setValue(0.500);

  dataWidget->geomLengthSpinBox = new QDoubleSpinBox;
  dataWidget->geomLengthSpinBox->setRange(-1000, 1000);
  dataWidget->geomLengthSpinBox->setSingleStep(0.01);
  dataWidget->geomLengthSpinBox->setDecimals(3);
  dataWidget->geomLengthSpinBox->setValue(1.000);

  QHBoxLayout *geomRLLayout = new QHBoxLayout;
  geomRLLayout->addWidget(geomRadiusLabel);
  geomRLLayout->addWidget(dataWidget->geomRadiusSpinBox);
  geomRLLayout->addWidget(dataWidget->geomLengthLabel);
  geomRLLayout->addWidget(dataWidget->geomLengthSpinBox);

  dataWidget->geomDimensionWidget = new QStackedWidget;

  QWidget *geomSizeWidget = new QWidget;
  geomSizeWidget->setLayout(geomSizeLayout);
  dataWidget->geomDimensionWidget->insertWidget(0, geomSizeWidget);

  QWidget *geomRLWidget = new QWidget;
  geomRLWidget->setLayout(geomRLLayout);
  dataWidget->geomDimensionWidget->insertWidget(1, geomRLWidget);
  dataWidget->geomDimensionWidget->setCurrentIndex(0);

  QLabel *laserRetroLabel = new QLabel(tr("laser retro: "));
  dataWidget->laserRetroSpinBox = new QDoubleSpinBox;
  dataWidget->laserRetroSpinBox->setRange(-1000, 1000);
  dataWidget->laserRetroSpinBox->setSingleStep(0.01);
  dataWidget->laserRetroSpinBox->setDecimals(3);
  dataWidget->laserRetroSpinBox->setValue(0.0);

  QLabel *maxContactsLabel = new QLabel(tr("max contacts: "));
  dataWidget->maxContactsSpinBox = new QSpinBox;
  dataWidget->maxContactsSpinBox->setRange(0, 1000);
  dataWidget->maxContactsSpinBox->setSingleStep(1);
  dataWidget->maxContactsSpinBox->setValue(10);

  QGridLayout *collisionGeneralLayout = new QGridLayout;
  collisionGeneralLayout->addWidget(nameLabel, 0, 0);
  collisionGeneralLayout->addWidget(dataWidget->collisionNameLabel, 0, 1);
  collisionGeneralLayout->addWidget(geometryLabel, 1, 0);
  collisionGeneralLayout->addWidget(dataWidget->geometryComboBox, 1, 1);
  collisionGeneralLayout->addWidget(dataWidget->geomDimensionWidget, 2, 1);

  QGridLayout *collisionPropertyLayout = new QGridLayout;
  collisionPropertyLayout->addWidget(laserRetroLabel, 0, 0);
  collisionPropertyLayout->addWidget(dataWidget->laserRetroSpinBox, 0, 1);
  collisionPropertyLayout->addWidget(maxContactsLabel, 1, 0);
  collisionPropertyLayout->addWidget(dataWidget->maxContactsSpinBox, 1, 1);

  QLabel *posXLabel = new QLabel(tr("x: "));
  QLabel *posYLabel = new QLabel(tr("y: "));
  QLabel *posZLabel = new QLabel(tr("z: "));
  QLabel *rotRLabel = new QLabel(tr("roll: "));
  QLabel *rotPLabel = new QLabel(tr("pitch: "));
  QLabel *rotYLabel = new QLabel(tr("yaw: "));

  dataWidget->posXSpinBox = new QDoubleSpinBox;
  dataWidget->posXSpinBox->setRange(-1000, 1000);
  dataWidget->posXSpinBox->setSingleStep(0.01);
  dataWidget->posXSpinBox->setDecimals(3);
  dataWidget->posXSpinBox->setValue(0.000);

  dataWidget->posYSpinBox = new QDoubleSpinBox;
  dataWidget->posYSpinBox->setRange(-1000, 1000);
  dataWidget->posYSpinBox->setSingleStep(0.01);
  dataWidget->posYSpinBox->setDecimals(3);
  dataWidget->posYSpinBox->setValue(0.000);

  dataWidget->posZSpinBox = new QDoubleSpinBox;
  dataWidget->posZSpinBox->setRange(-1000, 1000);
  dataWidget->posZSpinBox->setSingleStep(0.01);
  dataWidget->posZSpinBox->setDecimals(3);
  dataWidget->posZSpinBox->setValue(0.000);

  dataWidget->rotRSpinBox = new QDoubleSpinBox;
  dataWidget->rotRSpinBox->setRange(-1000, 1000);
  dataWidget->rotRSpinBox->setSingleStep(0.01);
  dataWidget->rotRSpinBox->setDecimals(3);
  dataWidget->rotRSpinBox->setValue(0.000);

  dataWidget->rotPSpinBox = new QDoubleSpinBox;
  dataWidget->rotPSpinBox->setRange(-1000, 1000);
  dataWidget->rotPSpinBox->setSingleStep(0.01);
  dataWidget->rotPSpinBox->setDecimals(3);
  dataWidget->rotPSpinBox->setValue(0.000);

  dataWidget->rotYSpinBox = new QDoubleSpinBox;
  dataWidget->rotYSpinBox->setRange(-1000, 1000);
  dataWidget->rotYSpinBox->setSingleStep(0.01);
  dataWidget->rotYSpinBox->setDecimals(3);
  dataWidget->rotYSpinBox->setValue(0.000);

  QGridLayout *poseGroupLayout = new QGridLayout;
  poseGroupLayout->addWidget(posXLabel, 0, 0);
  poseGroupLayout->addWidget(dataWidget->posXSpinBox, 0, 1);
  poseGroupLayout->addWidget(posYLabel, 0, 2);
  poseGroupLayout->addWidget(dataWidget->posYSpinBox, 0, 3);
  poseGroupLayout->addWidget(posZLabel, 0, 4);
  poseGroupLayout->addWidget(dataWidget->posZSpinBox, 0, 5);
  poseGroupLayout->addWidget(rotRLabel, 1, 0);
  poseGroupLayout->addWidget(dataWidget->rotRSpinBox, 1, 1);
  poseGroupLayout->addWidget(rotPLabel, 1, 2);
  poseGroupLayout->addWidget(dataWidget->rotPSpinBox, 1, 3);
  poseGroupLayout->addWidget(rotYLabel, 1, 4);
  poseGroupLayout->addWidget(dataWidget->rotYSpinBox, 1, 5);

  poseGroupLayout->setColumnStretch(1, 1);
  poseGroupLayout->setAlignment(dataWidget->posXSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(dataWidget->posYSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(dataWidget->posZSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(dataWidget->rotRSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(dataWidget->rotPSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(dataWidget->rotYSpinBox, Qt::AlignLeft);

  QGroupBox *poseGroupBox = new QGroupBox(tr("Pose"));
  poseGroupBox->setLayout(poseGroupLayout);

  collisionLayout->addLayout(collisionGeneralLayout);
  collisionLayout->addWidget(poseGroupBox);
  collisionLayout->addLayout(collisionPropertyLayout);
  collisionWidget->setLayout(collisionLayout);

  this->collisionsTreeWidget->setItemWidget(collisionChildItem, 0,
      collisionWidget);
  collisionItem->setExpanded(true);
  collisionChildItem->setExpanded(true);

  emit CollisionAdded();
}

/////////////////////////////////////////////////
void PartCollisionConfig::OnRemoveCollision(int _id)
{
  std::map<int, QTreeWidgetItem*>::iterator it = this->collisionItems.find(_id);
  if (it == this->collisionItems.end())
  {
    gzerr << "No collision item found" << std::endl;
    return;
  }
  QTreeWidgetItem *item = it->second;
  int index = this->collisionsTreeWidget->indexOfTopLevelItem(item);
  this->collisionsTreeWidget->takeTopLevelItem(index);

  this->collisionItems.erase(it);

  for (unsigned int i = 0; i < this->dataWidgets.size(); ++i)
  {
    if (this->dataWidgets[i]->id == _id)
    {
      emit CollisionRemoved(
          this->dataWidgets[i]->collisionNameLabel->text().toStdString());
      this->dataWidgets.erase(this->dataWidgets.begin() + i);
      break;
    }
  }
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetPose(unsigned int _index, const math::Pose &_pose)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->posXSpinBox->setValue(_pose.pos.x);
  this->dataWidgets[_index]->posYSpinBox->setValue(_pose.pos.y);
  this->dataWidgets[_index]->posZSpinBox->setValue(_pose.pos.z);

  this->dataWidgets[_index]->rotRSpinBox->setValue(_pose.rot.GetAsEuler().x);
  this->dataWidgets[_index]->rotPSpinBox->setValue(_pose.rot.GetAsEuler().y);
  this->dataWidgets[_index]->rotYSpinBox->setValue(_pose.rot.GetAsEuler().z);
}

/////////////////////////////////////////////////
math::Pose PartCollisionConfig::GetPose(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return math::Pose::Zero;
  }

  return math::Pose(this->dataWidgets[_index]->posXSpinBox->value(),
      this->dataWidgets[_index]->posYSpinBox->value(),
      this->dataWidgets[_index]->posZSpinBox->value(),
      this->dataWidgets[_index]->rotRSpinBox->value(),
      this->dataWidgets[_index]->rotPSpinBox->value(),
      this->dataWidgets[_index]->rotYSpinBox->value());
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetGeometry(unsigned int _index,
    const std::string &_geometry)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }
  std::string geometryStr = _geometry;
  if (geometryStr.substr(0, 5) == "unit_")
      geometryStr = geometryStr.substr(5);

  int dataIndex = this->dataWidgets[_index]->geometryComboBox->findText(
      QString(tr(geometryStr.c_str())));
  if (dataIndex >= 0)
  {
    this->dataWidgets[_index]->geometryComboBox->setCurrentIndex(dataIndex);
  }
}

/////////////////////////////////////////////////
std::string PartCollisionConfig::GetGeometry(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return NULL;
  }

  int dataIndex = this->dataWidgets[_index]->geometryComboBox->currentIndex();
  return "unit_" + this->dataWidgets[_index]->geometryComboBox->itemText(
      dataIndex).toStdString();
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetGeometrySize(unsigned int _index,
    const math::Vector3 &_size)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->geomSizeXSpinBox->setValue(_size.x);
  this->dataWidgets[_index]->geomSizeYSpinBox->setValue(_size.y);
  this->dataWidgets[_index]->geomSizeZSpinBox->setValue(_size.z);
}

/////////////////////////////////////////////////
math::Vector3 PartCollisionConfig::GetGeometrySize(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return math::Vector3::Zero;
  }

  return math::Vector3(this->dataWidgets[_index]->geomSizeXSpinBox->value(),
      this->dataWidgets[_index]->geomSizeYSpinBox->value(),
      this->dataWidgets[_index]->geomSizeZSpinBox->value());
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetGeometryRadius(unsigned int _index,
    double _radius)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->geomRadiusSpinBox->setValue(_radius);
}

/////////////////////////////////////////////////
double PartCollisionConfig::GetGeometryRadius(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return 0;
  }

  return this->dataWidgets[_index]->geomRadiusSpinBox->value();
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetGeometryLength(unsigned int _index,
    double _length)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->geomLengthSpinBox->setValue(_length);
}

/////////////////////////////////////////////////
double PartCollisionConfig::GetGeometryLength(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return 0;
  }

  return this->dataWidgets[_index]->geomLengthSpinBox->value();
}


/////////////////////////////////////////////////
void PartCollisionConfig::SetGeometryScale(unsigned int _index,
    const math::Vector3 &_scale)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  std::string geom =
      this->dataWidgets[_index]->geometryComboBox->currentText().toStdString();

  if (geom == "box")
  {
    this->SetGeometrySize(_index, _scale);
  }
  else if (geom == "cylinder")
  {
    this->SetGeometryRadius(_index, _scale[0]/2);
    this->SetGeometryLength(_index, _scale[2]);
  }
  else if (geom == "sphere")
  {
    this->SetGeometryRadius(_index, _scale[0]/2);
  }
}

/////////////////////////////////////////////////
math::Vector3 PartCollisionConfig::GetGeometryScale(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return math::Vector3::Zero;
  }

  math::Vector3 scale;
  std::string geom =
      this->dataWidgets[_index]->geometryComboBox->currentText().toStdString();

  if (geom == "box")
  {
    scale = this->GetGeometrySize(_index);
  }
  else if (geom == "cylinder")
  {
    double r = this->GetGeometryRadius(_index);
    scale = math::Vector3(r*2, r*2, this->GetGeometryLength(_index));
  }
  else if (geom == "sphere")
  {
    double r = this->GetGeometryRadius(_index);
    scale = math::Vector3(r*2, r*2, r*2);
  }
  return scale;
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetName(unsigned int _index, const std::string &_name)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->collisionNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
std::string PartCollisionConfig::GetName(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return "";
  }

  return this->dataWidgets[_index]->collisionNameLabel->text().toStdString();
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetLaserRetro(unsigned int _index,
    double _retro)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->laserRetroSpinBox->setValue(_retro);
}

/////////////////////////////////////////////////
double PartCollisionConfig::GetLaserRetro(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return 0;
  }

  return this->dataWidgets[_index]->laserRetroSpinBox->value();
}

/////////////////////////////////////////////////
void PartCollisionConfig::SetMaxContacts(unsigned int _index,
    double _maxContacts)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->maxContactsSpinBox->setValue(_maxContacts);
}

/////////////////////////////////////////////////
double PartCollisionConfig::GetMaxContacts(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return 0;
  }

  return this->dataWidgets[_index]->maxContactsSpinBox->value();
}

/////////////////////////////////////////////////
void CollisionDataWidget::GeometryChanged(const QString _text)
{
  QWidget *widget= qobject_cast<QWidget *>(QObject::sender());

  if (widget)
  {
    std::string textStr = _text.toStdString();
    if (textStr == "box")
    {
      this->geomDimensionWidget->setCurrentIndex(0);
    }
    else if (textStr == "cylinder")
    {
      this->geomDimensionWidget->setCurrentIndex(1);
      this->geomLengthSpinBox->show();
      this->geomLengthLabel->show();
    }
    else if (textStr == "sphere")
    {
      this->geomDimensionWidget->setCurrentIndex(1);
      this->geomLengthSpinBox->hide();
      this->geomLengthLabel->hide();
    }
  }
}*/
