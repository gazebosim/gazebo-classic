/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/gui/model/CollisionConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
CollisionConfig::CollisionConfig()
{
  this->setObjectName("CollisionConfig");

  // Layout for list
  this->listLayout = new QVBoxLayout();
  this->listLayout->setContentsMargins(0, 0, 0, 0);
  this->listLayout->setAlignment(Qt::AlignTop);

  // Widget for list, which will be scrollable
  QWidget *listWidget = new QWidget();
  listWidget->setLayout(this->listLayout);
  listWidget->setStyleSheet("QWidget{background-color: #808080}");

  // Scroll area for list
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(listWidget);
  scrollArea->setWidgetResizable(true);

  // Add Collision button
  QPushButton *addCollisionButton = new QPushButton(tr("+ &Another Collision"));
  addCollisionButton->setMaximumWidth(200);
  addCollisionButton->setDefault(false);
  addCollisionButton->setAutoDefault(false);
  connect(addCollisionButton, SIGNAL(clicked()), this, SLOT(OnAddCollision()));

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(scrollArea);
  mainLayout->addWidget(addCollisionButton);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);

  this->counter = 0;
  this->signalMapper = new QSignalMapper(this);

  connect(this->signalMapper, SIGNAL(mapped(int)),
     this, SLOT(OnRemoveCollision(int)));
}

/////////////////////////////////////////////////
CollisionConfig::~CollisionConfig()
{
  while (!this->configs.empty())
  {
    auto config = this->configs.begin();
    delete config->second;
    this->configs.erase(config);
  }
  while (!this->deletedConfigs.empty())
  {
    auto config = this->deletedConfigs.begin();
    delete config->second;
    this->deletedConfigs.erase(config);
  }
}

/////////////////////////////////////////////////
void CollisionConfig::Init()
{
  // Keep original data in case user cancels
  for (auto &it : this->configs)
  {
    CollisionConfigData *configData = it.second;
    configData->originalDataMsg.CopyFrom(*this->GetData(configData->name));
  }

  // Clear lists
  for (auto &it : this->deletedConfigs)
    delete it.second->widget;
  this->deletedConfigs.clear();

  this->addedConfigs.clear();
}

/////////////////////////////////////////////////
void CollisionConfig::OnAddCollision()
{
  std::stringstream collisionIndex;
  collisionIndex << "collision_" << this->counter;
  this->AddCollision(collisionIndex.str());
  emit CollisionAdded(collisionIndex.str());
}

/////////////////////////////////////////////////
unsigned int CollisionConfig::GetCollisionCount() const
{
  return this->configs.size();
}

/////////////////////////////////////////////////
void CollisionConfig::Reset()
{
  for (auto &it : this->configs)
  {
    this->listLayout->removeWidget(it.second->widget);
    delete it.second;
  }

  this->configs.clear();
}

/////////////////////////////////////////////////
void CollisionConfig::UpdateCollision(const std::string &_name,
    ConstCollisionPtr _collisionMsg)
{
  for (auto &it : this->configs)
  {
    if (it.second->name == _name)
    {
      CollisionConfigData *configData = it.second;
      configData->configWidget->UpdateFromMsg(_collisionMsg.get());
      break;
    }
  }
}

/////////////////////////////////////////////////
void CollisionConfig::AddCollision(const std::string &_name,
    const msgs::Collision *_collisionMsg)
{
  // Header button
  QRadioButton *headerButton = new QRadioButton();
  headerButton->setChecked(false);
  headerButton->setFocusPolicy(Qt::NoFocus);
  headerButton->setText(QString(_name.c_str()));
  headerButton->setStyleSheet(
     "QRadioButton {\
        color: #d0d0d0;\
      }\
      QRadioButton::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QRadioButton::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");

  // Remove button
  QToolButton *removeCollisionButton = new QToolButton(this);
  removeCollisionButton->setObjectName(
      "removeCollisionButton_" + QString::number(this->counter));
  removeCollisionButton->setFixedSize(QSize(30, 30));
  removeCollisionButton->setToolTip("Remove " + QString(_name.c_str()));
  removeCollisionButton->setIcon(QPixmap(":/images/trashcan.png"));
  removeCollisionButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  removeCollisionButton->setIconSize(QSize(16, 16));
  removeCollisionButton->setCheckable(false);
  connect(removeCollisionButton, SIGNAL(clicked()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(removeCollisionButton, this->counter);

  // Header Layout
  QHBoxLayout *headerLayout = new QHBoxLayout;
  headerLayout->setContentsMargins(0, 0, 0, 0);
  headerLayout->addWidget(headerButton);
  headerLayout->addWidget(removeCollisionButton);

  // Header widget
  QWidget *headerWidget = new QWidget;
  headerWidget->setLayout(headerLayout);

  // ConfigWidget
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
  if (!surfaceMsg->has_collide_bitmask())
    surfaceMsg->set_collide_bitmask(1);

  msgs::Friction *frictionMsg = surfaceMsg->mutable_friction();
  if (!frictionMsg->has_mu())
    frictionMsg->set_mu(1.0);
  if (!frictionMsg->has_mu2())
    frictionMsg->set_mu2(1.0);

  msgs::Friction::Torsional *torsionalMsg =
      frictionMsg->mutable_torsional();
  if (!torsionalMsg->has_coefficient())
    torsionalMsg->set_coefficient(1.0);
  if (!torsionalMsg->has_use_patch_radius())
    torsionalMsg->set_use_patch_radius(true);
  if (!torsionalMsg->has_patch_radius())
    torsionalMsg->set_patch_radius(0.0);
  if (!torsionalMsg->has_surface_radius())
    torsionalMsg->set_surface_radius(0.0);

  ConfigWidget *configWidget = new ConfigWidget;
  configWidget->Load(&msgToLoad);
  configWidget->hide();

  configWidget->SetWidgetVisible("id", false);
  configWidget->SetWidgetVisible("name", false);
  configWidget->SetWidgetReadOnly("id", true);
  configWidget->SetWidgetReadOnly("name", true);

  connect(configWidget, SIGNAL(PoseValueChanged(const QString &,
      const ignition::math::Pose3d &)), this,
      SLOT(OnPoseChanged(const QString &, const ignition::math::Pose3d &)));

  connect(configWidget, SIGNAL(GeometryValueChanged(const std::string &,
      const std::string &, const ignition::math::Vector3d &,
      const std::string &)), this, SLOT(OnGeometryChanged(const std::string &,
      const std::string &, const ignition::math::Vector3d &,
      const std::string &)));

  // Item layout
  QVBoxLayout *itemLayout = new QVBoxLayout();
  itemLayout->addWidget(headerWidget);
  itemLayout->addWidget(configWidget);

  // Put the layout in a widget which can be added/deleted
  QWidget *item = new QWidget();
  item->setLayout(itemLayout);

  // Add to the list
  this->listLayout->addWidget(item);

  // Fill ConfigData
  CollisionConfigData *configData = new CollisionConfigData;
  configData->configWidget = configWidget;
  configData->id =  this->counter;
  configData->widget = item;
  configData->name = _name;

  this->connect(headerButton, SIGNAL(toggled(bool)), configData,
      SLOT(OnToggleItem(bool)));

  this->connect(configWidget, SIGNAL(GeometryChanged()), configData,
      SLOT(OnGeometryChanged()));

  this->connect(configData, SIGNAL(CollisionChanged(
      const std::string &, const std::string &)),
      this, SLOT(OnCollisionChanged(
      const std::string &, const std::string &)));

  this->configs[this->counter] = configData;
  this->addedConfigs[this->counter] = configData;

  this->counter++;
}

/////////////////////////////////////////////////
void CollisionConfig::OnRemoveCollision(int _id)
{
  auto it = this->configs.find(_id);
  if (it == this->configs.end())
  {
    gzerr << "Collision not found " << std::endl;
    return;
  }

  CollisionConfigData *configData = this->configs[_id];

  // Ask for confirmation
  // Don't open the dialog during tests, see issue #1963
  auto inTest = getenv("IN_TESTSUITE");
  if (!inTest)
  {
    std::string msg;

    if (this->configs.size() == 1)
    {
      msg = "Are you sure you want to remove " +
          configData->name + "?\n\n" +
          "This is the only collision. \n" +
          "Without collisions, this link won't collide with anything.\n";
    }
    else
    {
      msg = "Are you sure you want to remove " +
          configData->name + "?\n";
    }

    QMessageBox msgBox(QMessageBox::Warning, QString("Remove collision?"),
        QString(msg.c_str()));
    msgBox.setWindowFlags(Qt::Window | Qt::WindowTitleHint |
        Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

    QPushButton *cancelButton =
        msgBox.addButton("Cancel", QMessageBox::RejectRole);
    QPushButton *removeButton = msgBox.addButton("Remove",
        QMessageBox::AcceptRole);
    msgBox.setDefaultButton(removeButton);
    msgBox.setEscapeButton(cancelButton);
    msgBox.exec();
    if (msgBox.clickedButton() && msgBox.clickedButton() != removeButton)
      return;
  }

  // Remove
  this->listLayout->removeWidget(configData->widget);
  configData->widget->hide();

  emit CollisionRemoved(configData->name);

  // Add to delete list only if this existed from the beginning
  auto itAdded = this->addedConfigs.find(_id);
  if (itAdded == this->addedConfigs.end())
  {
    this->deletedConfigs[_id] = configData;
    this->configs.erase(it);
  }
  else
  {
    this->addedConfigs.erase(itAdded);
  }
}

/////////////////////////////////////////////////
msgs::Collision *CollisionConfig::GetData(const std::string &_name) const
{
  for (auto const &it : this->configs)
  {
    std::string name = it.second->name;
    if (name == _name)
    {
      return dynamic_cast<msgs::Collision *>(it.second->configWidget->Msg());
    }
  }
  return NULL;
}

/////////////////////////////////////////////////
void CollisionConfig::SetGeometry(const std::string &_name,
    const ignition::math::Vector3d &_size, const std::string &_uri)
{
  for (auto const &it : this->configs)
  {
    if (it.second->name == _name)
    {
      ignition::math::Vector3d dimensions;
      std::string uri;
      std::string type = it.second->configWidget->GeometryWidgetValue(
          "geometry", dimensions, uri);
      it.second->configWidget->SetGeometryWidgetValue("geometry", type,
          _size, _uri);
      break;
    }
  }
}

/////////////////////////////////////////////////
void CollisionConfig::Geometry(const std::string &_name,
    ignition::math::Vector3d &_size, std::string &_uri) const
{
  for (auto const &it : this->configs)
  {
    if (it.second->name == _name)
    {
      it.second->configWidget->GeometryWidgetValue("geometry", _size, _uri);
      break;
    }
  }
}

/////////////////////////////////////////////////
const std::map<int, CollisionConfigData *> &CollisionConfig::ConfigData() const
{
  return this->configs;
}

/////////////////////////////////////////////////
void CollisionConfig::OnPoseChanged(const QString &/*_name*/,
    const ignition::math::Pose3d &/*_value*/)
{
  emit Applied();
}

/////////////////////////////////////////////////
void CollisionConfig::OnGeometryChanged(const std::string &/*_name*/,
    const std::string &/*_value*/,
    const ignition::math::Vector3d &/*dimensions*/,
    const std::string &/*_uri*/)
{
  emit Applied();
}

/////////////////////////////////////////////////
void CollisionConfig::RestoreOriginalData()
{
  // Remove added configs
  for (auto &it : this->addedConfigs)
  {
    auto configData = it.second;

    this->listLayout->removeWidget(configData->widget);
    delete configData->widget;

    emit CollisionRemoved(configData->name);

    this->configs.erase(it.first);
  }
  this->addedConfigs.clear();

  // Restore previously existing configs
  for (auto &it : this->configs)
  {
    it.second->RestoreOriginalData();
  }

  // Reinsert deleted configs
  for (auto &it : this->deletedConfigs)
  {
    this->listLayout->addWidget(it.second->widget);
    it.second->widget->show();

    this->configs[it.first] = it.second;
    emit CollisionAdded(it.second->name);
  }
  this->deletedConfigs.clear();
}

/////////////////////////////////////////////////
void CollisionConfig::OnCollisionChanged(const std::string &_name,
    const std::string &_type)
{
  emit CollisionChanged(_name, _type);
}

/////////////////////////////////////////////////
void CollisionConfigData::OnToggleItem(bool _checked)
{
  if (_checked)
    this->configWidget->show();
  else
    this->configWidget->hide();
}

/////////////////////////////////////////////////
void CollisionConfigData::OnGeometryChanged()
{
  emit CollisionChanged(this->name, "geometry");
}

/////////////////////////////////////////////////
void CollisionConfigData::RestoreOriginalData()
{
  msgs::CollisionPtr collisionPtr;
  collisionPtr.reset(new msgs::Collision);
  collisionPtr->CopyFrom(this->originalDataMsg);

  // Update default widgets
  this->configWidget->blockSignals(true);
  this->configWidget->UpdateFromMsg(collisionPtr.get());
  this->configWidget->blockSignals(false);
}
