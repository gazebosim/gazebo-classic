/*
 * Copyright 2014 Open Source Robotics Foundation
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
#include "gazebo/gui/model/PartCollisionTab.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PartCollisionTab::PartCollisionTab()
{
  this->setObjectName("partCollisionTab");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->collisionsTreeWidget = new QTreeWidget();
  this->collisionsTreeWidget->setColumnCount(1);
  this->collisionsTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->collisionsTreeWidget->header()->hide();

  this->collisionsTreeWidget->setSelectionMode(QAbstractItemView::NoSelection);
  connect(this->collisionsTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
      this, SLOT(OnItemSelection(QTreeWidgetItem *, int)));

  QPushButton *AddCollisionButton = new QPushButton(tr("+ &Another Collision"));
  connect(AddCollisionButton, SIGNAL(clicked()), this, SLOT(OnAddCollision()));

  mainLayout->addWidget(this->collisionsTreeWidget);
  mainLayout->addWidget(AddCollisionButton);
  this->setLayout(mainLayout);

  this->counter = 0;
  this->signalMapper = new QSignalMapper(this);

  connect(this->signalMapper, SIGNAL(mapped(int)),
     this, SLOT(OnRemoveCollision(int)));
}

/////////////////////////////////////////////////
PartCollisionTab::~PartCollisionTab()
{
}

/////////////////////////////////////////////////
void PartCollisionTab::AddCollision()
{
  this->OnAddCollision();
}

/////////////////////////////////////////////////
unsigned int PartCollisionTab::GetCollisionCount() const
{
  return this->dataWidgets.size();
}

/////////////////////////////////////////////////
void PartCollisionTab::Reset()
{
  this->collisionItems.clear();
  this->dataWidgets.clear();
  this->collisionsTreeWidget->clear();
}

/////////////////////////////////////////////////
void PartCollisionTab::OnAddCollision()
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
  dataWidget->geometryComboBox->addItem(tr("unit_box"));
  dataWidget->geometryComboBox->addItem(tr("unit_cylinder"));
  dataWidget->geometryComboBox->addItem(tr("unit_sphere"));
  // geometryComboBox->addItem(tr("custom"));

  QGridLayout *collisionGeneralLayout = new QGridLayout;
  collisionGeneralLayout->addWidget(nameLabel, 0, 0);
  collisionGeneralLayout->addWidget(dataWidget->collisionNameLabel, 0, 1);
  collisionGeneralLayout->addWidget(geometryLabel, 1, 0);
  collisionGeneralLayout->addWidget(dataWidget->geometryComboBox, 1, 1);

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
  collisionWidget->setLayout(collisionLayout);

  this->collisionsTreeWidget->setItemWidget(collisionChildItem, 0, collisionWidget);
  collisionItem->setExpanded(true);
  collisionChildItem->setExpanded(true);

  emit CollisionAdded();
}

/////////////////////////////////////////////////
void PartCollisionTab::OnItemSelection(QTreeWidgetItem *_item,
                                         int /*_column*/)
{
  if (_item && _item->childCount() > 0)
    _item->setExpanded(!_item->isExpanded());
}

/////////////////////////////////////////////////
void PartCollisionTab::OnRemoveCollision(int _id)
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
void PartCollisionTab::SetPose(unsigned int _index, const math::Pose &_pose)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
  }

  this->dataWidgets[_index]->posXSpinBox->setValue(_pose.pos.x);
  this->dataWidgets[_index]->posYSpinBox->setValue(_pose.pos.y);
  this->dataWidgets[_index]->posZSpinBox->setValue(_pose.pos.z);

  this->dataWidgets[_index]->rotRSpinBox->setValue(_pose.rot.GetAsEuler().x);
  this->dataWidgets[_index]->rotPSpinBox->setValue(_pose.rot.GetAsEuler().y);
  this->dataWidgets[_index]->rotYSpinBox->setValue(_pose.rot.GetAsEuler().z);
}

/////////////////////////////////////////////////
math::Pose PartCollisionTab::GetPose(unsigned int _index) const
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
void PartCollisionTab::SetGeometry(unsigned int _index,
    const std::string &_geometry)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
  }
  int dataIndex = this->dataWidgets[_index]->geometryComboBox->findText(
      QString(tr(_geometry.c_str())));
  if (dataIndex >= 0)
  {
    this->dataWidgets[_index]->geometryComboBox->setCurrentIndex(dataIndex);
  }
}

/////////////////////////////////////////////////
std::string PartCollisionTab::GetGeometry(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return NULL;
  }

  int dataIndex = this->dataWidgets[_index]->geometryComboBox->currentIndex();
  return this->dataWidgets[_index]->geometryComboBox->itemText(
      dataIndex).toStdString();
}

/////////////////////////////////////////////////
void PartCollisionTab::SetName(unsigned int _index, const std::string &_name)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
  }

  this->dataWidgets[_index]->collisionNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
std::string PartCollisionTab::GetName(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return "";
  }

  return this->dataWidgets[_index]->collisionNameLabel->text().toStdString();
}
