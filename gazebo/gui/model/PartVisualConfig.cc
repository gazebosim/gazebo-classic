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
  this->visualsTreeWidget->setIndentation(5);

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
//  return this->dataWidgets.size();
  return this->configs.size();
}

/////////////////////////////////////////////////
void PartVisualConfig::Reset()
{
//  this->visualItems.clear();
  // this->dataWidgets.clear();
//  this->configWidgets.clear();
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

//  this->visualItems[counter] = visualItem;

  QWidget *visualItemWidget = new QWidget;
  QHBoxLayout *visualItemLayout = new QHBoxLayout;
  QLabel *visualLabel = new QLabel(QString(_name.c_str()));

  QPushButton *removeVisualButton = new QPushButton(tr("Remove"));
  connect(removeVisualButton, SIGNAL(clicked()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(removeVisualButton, this->counter);

/*  VisualDataWidget *dataWidget = new VisualDataWidget;
  dataWidget->id = counter;
  this->dataWidgets.push_back(dataWidget);
  counter++;*/

  visualItemLayout->addWidget(visualLabel);
  visualItemLayout->addWidget(removeVisualButton);
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

/*  QLabel *nameLabel = new QLabel(tr("Name:"));
  dataWidget->visualNameLabel = new QLabel(tr(""));

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

  QLabel *transparencyLabel = new QLabel(tr("Transparency:"));
  dataWidget->transparencySpinBox = new QDoubleSpinBox;
  dataWidget->transparencySpinBox->setRange(-1000, 1000);
  dataWidget->transparencySpinBox->setSingleStep(0.01);
  dataWidget->transparencySpinBox->setDecimals(3);
  dataWidget->transparencySpinBox->setValue(0.000);

  QLabel *materialLabel = new QLabel(tr("Material:"));
  dataWidget->materialLineEdit = new QLineEdit;

  QGridLayout *visualGeneralLayout = new QGridLayout;
  visualGeneralLayout->addWidget(nameLabel, 0, 0);
  visualGeneralLayout->addWidget(dataWidget->visualNameLabel, 0, 1);
  visualGeneralLayout->addWidget(geometryLabel, 1, 0);
  visualGeneralLayout->addWidget(dataWidget->geometryComboBox, 1, 1);
  visualGeneralLayout->addWidget(dataWidget->geomDimensionWidget, 2, 1);

  QGridLayout *visualPropertyLayout = new QGridLayout;
  visualPropertyLayout->addWidget(transparencyLabel, 0, 0);
  visualPropertyLayout->addWidget(dataWidget->transparencySpinBox, 0, 1);
  visualPropertyLayout->addWidget(materialLabel, 1, 0);
  visualPropertyLayout->addWidget(dataWidget->materialLineEdit, 1, 1);

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

  visualLayout->addLayout(visualGeneralLayout);
  visualLayout->addWidget(poseGroupBox);
  visualLayout->addLayout(visualPropertyLayout);*/
  visualWidget->setLayout(visualLayout);
  visualWidget->setContentsMargins(0, 0, 0, 0);

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
//      std::cerr << " update visual  "<< _visualMsg->DebugString()
//          << std::endl;
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
  /*std::map<int, QTreeWidgetItem*>::iterator it = this->visualItems.find(_id);
  if (it == this->visualItems.end())
  {
    gzerr << "No visual item found" << std::endl;
    return;
  }
  QTreeWidgetItem *item = it->second;*/

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

//  this->visualItems.erase(it);
  emit VisualRemoved(this->configs[_id]->name);
  this->configs.erase(it);

/*  for (unsigned int i = 0; i < this->configs.size(); ++i)
  {
    if (this->configWidgets[i]->property("id").toInt() == _id)
    {
      emit VisualRemoved(
          this->configWidgets[i]->visualNameLabel->text().toStdString());
      this->configWidgets.erase(this->configWidgets.begin() + i);
      break;
    }
  }*/
}

/////////////////////////////////////////////////
msgs::Visual *PartVisualConfig::GetData(const std::string &_name) const
{

  std::map<int, VisualConfigData *>::const_iterator it;
  for (it = this->configs.begin(); it != this->configs.end(); ++it)
  {
    std::string name = it->second->name;
    if (name == _name)
      return dynamic_cast<msgs::Visual *>(it->second->configWidget->GetMsg());
  }
  return NULL;
}



/*/////////////////////////////////////////////////
void PartVisualConfig::SetPose(unsigned int _index, const math::Pose &_pose)
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
math::Pose PartVisualConfig::GetPose(unsigned int _index) const
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
void PartVisualConfig::SetTransparency(unsigned int _index, double _transparency)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->transparencySpinBox->setValue(_transparency);
}

/////////////////////////////////////////////////
double PartVisualConfig::GetTransparency(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return -1;
  }

  return this->dataWidgets[_index]->transparencySpinBox->value();
}

/////////////////////////////////////////////////
std::string PartVisualConfig::GetMaterial(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return "";
  }

  return this->dataWidgets[_index]->materialLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
void PartVisualConfig::SetMaterial(unsigned int _index,
    const std::string &_material)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->materialLineEdit->setText(tr(_material.c_str()));
}


/////////////////////////////////////////////////
void PartVisualConfig::SetGeometry(unsigned int _index,
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
std::string PartVisualConfig::GetGeometry(unsigned int _index) const
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
void PartVisualConfig::SetGeometrySize(unsigned int _index,
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
math::Vector3 PartVisualConfig::GetGeometrySize(unsigned int _index) const
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
void PartVisualConfig::SetGeometryRadius(unsigned int _index,
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
double PartVisualConfig::GetGeometryRadius(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return 0;
  }

  return this->dataWidgets[_index]->geomRadiusSpinBox->value();
}

/////////////////////////////////////////////////
void PartVisualConfig::SetGeometryLength(unsigned int _index,
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
double PartVisualConfig::GetGeometryLength(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return 0;
  }

  return this->dataWidgets[_index]->geomLengthSpinBox->value();
}


/////////////////////////////////////////////////
void PartVisualConfig::SetGeometryScale(unsigned int _index,
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
math::Vector3 PartVisualConfig::GetGeometryScale(unsigned int _index) const
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
void PartVisualConfig::SetName(unsigned int _index, const std::string &_name)
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return;
  }

  this->dataWidgets[_index]->visualNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
std::string PartVisualConfig::GetName(unsigned int _index) const
{
  if (_index >= this->dataWidgets.size())
  {
    gzerr << "Index is out of range" << std::endl;
    return "";
  }

  return this->dataWidgets[_index]->visualNameLabel->text().toStdString();
}

/////////////////////////////////////////////////
void VisualDataWidget::GeometryChanged(const QString _text)
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
