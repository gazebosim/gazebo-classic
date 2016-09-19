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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/rendering/Material.hh"
#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/VisualConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
VisualConfig::VisualConfig()
{
  this->setObjectName("VisualConfig");

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

  // Add Visual button
  QPushButton *addVisualButton = new QPushButton(tr("+ &Another Visual"));
  addVisualButton->setMaximumWidth(200);
  addVisualButton->setDefault(false);
  addVisualButton->setAutoDefault(false);
  connect(addVisualButton, SIGNAL(clicked()), this, SLOT(OnAddVisual()));

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(scrollArea);
  mainLayout->addWidget(addVisualButton);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);

  this->counter = 0;
  this->signalMapper = new QSignalMapper(this);

  connect(this->signalMapper, SIGNAL(mapped(int)),
     this, SLOT(OnRemoveVisual(int)));
}

/////////////////////////////////////////////////
VisualConfig::~VisualConfig()
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
void VisualConfig::Init()
{
  // Keep original data in case user cancels
  for (auto &it : this->configs)
  {
    VisualConfigData *configData = it.second;
    configData->originalDataMsg.CopyFrom(*this->GetData(configData->name));
  }

  // Clear lists
  for (auto &it : this->deletedConfigs)
    delete it.second->widget;
  this->deletedConfigs.clear();

  this->addedConfigs.clear();
}

/////////////////////////////////////////////////
void VisualConfig::OnAddVisual()
{
  std::stringstream visualIndex;
  visualIndex << "visual_" << this->counter;
  this->AddVisual(visualIndex.str());
  emit VisualAdded(visualIndex.str());
}

/////////////////////////////////////////////////
unsigned int VisualConfig::GetVisualCount() const
{
  return this->configs.size();
}

/////////////////////////////////////////////////
void VisualConfig::Reset()
{
  for (auto &it : this->configs)
  {
    this->listLayout->removeWidget(it.second->widget);
    delete it.second;
  }

  this->configs.clear();
}

/////////////////////////////////////////////////
void VisualConfig::AddVisual(const std::string &_name,
    const msgs::Visual *_visualMsg)
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
  QToolButton *removeVisualButton = new QToolButton(this);
  removeVisualButton->setObjectName(
      "removeVisualButton_" + QString::number(this->counter));
  removeVisualButton->setFixedSize(QSize(30, 30));
  removeVisualButton->setToolTip("Remove " + QString(_name.c_str()));
  removeVisualButton->setIcon(QPixmap(":/images/trashcan.png"));
  removeVisualButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  removeVisualButton->setIconSize(QSize(16, 16));
  removeVisualButton->setCheckable(false);
  connect(removeVisualButton, SIGNAL(clicked()), this->signalMapper,
      SLOT(map()));
  this->signalMapper->setMapping(removeVisualButton, this->counter);

  // Header Layout
  QHBoxLayout *headerLayout = new QHBoxLayout;
  headerLayout->setContentsMargins(0, 0, 0, 0);
  headerLayout->addWidget(headerButton);
  headerLayout->addWidget(removeVisualButton);

  // Header widget
  QWidget *headerWidget = new QWidget;
  headerWidget->setLayout(headerLayout);

  // ConfigWidget
  msgs::Visual msgToLoad;
  if (_visualMsg)
    msgToLoad = *_visualMsg;

  // set default values
  // TODO: auto-fill them with SDF defaults
  msgs::Material *matMsg = msgToLoad.mutable_material();
  if (!matMsg->has_lighting())
      matMsg->set_lighting(true);

  ConfigWidget *configWidget = new ConfigWidget;
  configWidget->Load(&msgToLoad);
  configWidget->hide();

  configWidget->SetWidgetVisible("id", false);
  configWidget->SetWidgetVisible("name", false);
  configWidget->SetWidgetVisible("parent_name", false);
  configWidget->SetWidgetVisible("parent_id", false);
  configWidget->SetWidgetVisible("delete_me", false);
  configWidget->SetWidgetVisible("is_static", false);
  configWidget->SetWidgetVisible("visible", false);
  configWidget->SetWidgetVisible("scale", false);
  configWidget->SetWidgetVisible("plugin", false);
  configWidget->SetWidgetVisible("type", false);
  configWidget->SetWidgetReadOnly("id", true);
  configWidget->SetWidgetReadOnly("name", true);
  configWidget->SetWidgetReadOnly("parent_name", true);
  configWidget->SetWidgetReadOnly("parent_id", true);
  configWidget->SetWidgetReadOnly("delete_me", true);
  configWidget->SetWidgetReadOnly("is_static", true);
  configWidget->SetWidgetReadOnly("visible", true);
  configWidget->SetWidgetReadOnly("scale", true);
  configWidget->SetWidgetReadOnly("plugin", true);
  configWidget->SetWidgetReadOnly("type", true);

  // Connect config widget signals
  connect(configWidget, SIGNAL(PoseValueChanged(const QString &,
      const ignition::math::Pose3d &)), this,
      SLOT(OnPoseChanged(const QString &, const ignition::math::Pose3d &)));

  connect(configWidget, SIGNAL(GeometryValueChanged(const std::string &,
      const std::string &, const ignition::math::Vector3d &,
      const std::string &)), this, SLOT(OnGeometryChanged(const std::string &,
      const std::string &, const ignition::math::Vector3d &,
      const std::string &)));

  connect(configWidget, SIGNAL(ColorValueChanged(const QString &,
      const gazebo::common::Color &)), this,
      SLOT(OnColorChanged(const QString &, const gazebo::common::Color &)));

  connect(configWidget, SIGNAL(DoubleValueChanged(const QString &,
      const double)), this,
      SLOT(OnDoubleChanged(const QString &, const double)));

  connect(configWidget, SIGNAL(StringValueChanged(const QString &,
      const std::string &)), this,
      SLOT(OnStringChanged(const QString &, const std::string &)));

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
  VisualConfigData *configData = new VisualConfigData;
  configData->configWidget = configWidget;
  configData->id =  this->counter;
  configData->widget = item;
  configData->name = _name;
  connect(headerButton, SIGNAL(toggled(bool)), configData,
           SLOT(OnToggleItem(bool)));
  this->configs[this->counter] = configData;
  this->addedConfigs[this->counter] = configData;

  this->counter++;
}

/////////////////////////////////////////////////
void VisualConfig::UpdateVisual(const std::string &_name,
    ConstVisualPtr _visualMsg)
{
  for (auto &it : this->configs)
  {
    if (it.second->name == _name)
    {
      VisualConfigData *configData = it.second;
      configData->configWidget->UpdateFromMsg(_visualMsg.get());
      break;
    }
  }
}

/////////////////////////////////////////////////
void VisualConfig::OnRemoveVisual(int _id)
{
  auto it = this->configs.find(_id);
  if (it == this->configs.end())
  {
    gzerr << "Visual not found " << std::endl;
    return;
  }

  VisualConfigData *configData = this->configs[_id];

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
          "This is the only visual. \n" +
          "Without visuals, this link won't be visible.\n";
    }
    else
    {
      msg = "Are you sure you want to remove " +
          configData->name + "?\n";
    }

    QMessageBox msgBox(QMessageBox::Warning, QString("Remove visual?"),
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

  emit VisualRemoved(configData->name);
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
msgs::Visual *VisualConfig::GetData(const std::string &_name) const
{
  for (auto const &it : this->configs)
  {
    if (it.second->name == _name)
      return dynamic_cast<msgs::Visual *>(it.second->configWidget->Msg());
  }
  return NULL;
}

/////////////////////////////////////////////////
void VisualConfig::SetGeometry(const std::string &_name,
    const ignition::math::Vector3d &_size, const std::string &_uri)
{
  for (auto &it : this->configs)
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
void VisualConfig::Geometry(const std::string &_name,
    ignition::math::Vector3d &_size, std::string &_uri)
{
  for (auto &it : this->configs)
  {
    if (it.second->name == _name)
    {
      it.second->configWidget->GeometryWidgetValue("geometry", _size, _uri);
      break;
    }
  }
}

/////////////////////////////////////////////////
void VisualConfig::SetMaterial(const std::string &_name,
  const std::string &_materialName, const common::Color &_ambient,
  const common::Color &_diffuse, const common::Color &_specular,
  const common::Color &_emissive)
{
  for (auto &it : this->configs)
  {
    if (it.second->name == _name)
    {
      it.second->configWidget->SetStringWidgetValue("material::script::name",
          _materialName);
      it.second->configWidget->SetColorWidgetValue("material::ambient",
          _ambient);
      it.second->configWidget->SetColorWidgetValue("material::diffuse",
          _diffuse);
      it.second->configWidget->SetColorWidgetValue("material::specular",
          _specular);
      it.second->configWidget->SetColorWidgetValue("material::emissive",
          _emissive);
      break;
    }
  }
}

/////////////////////////////////////////////////
const std::map<int, VisualConfigData *> &VisualConfig::ConfigData() const
{
  return this->configs;
}

/////////////////////////////////////////////////
void VisualConfig::OnPoseChanged(const QString &/*_name*/,
    const ignition::math::Pose3d &/*_value*/)
{
  emit Applied();
}

/////////////////////////////////////////////////
void VisualConfig::OnGeometryChanged(const std::string &/*_name*/,
    const std::string &/*_value*/,
    const ignition::math::Vector3d &/*dimensions*/,
    const std::string &/*_uri*/)
{
  emit Applied();
}

/////////////////////////////////////////////////
void VisualConfig::OnColorChanged(const QString &/*_name*/,
    const gazebo::common::Color &/*_value*/)
{
  emit Applied();
}

/////////////////////////////////////////////////
void VisualConfig::OnDoubleChanged(const QString &_name,
    const double /*_value*/)
{
  // Only transparency affects the visualization
  if (_name == "transparency")
    emit Applied();
}

/////////////////////////////////////////////////
void VisualConfig::OnStringChanged(const QString &_name,
    const std::string &/*_value*/)
{
  // Only material script affects the visualization
  if (_name == "material::script::name")
    emit Applied();
}

/////////////////////////////////////////////////
void VisualConfig::RestoreOriginalData()
{
  // Remove added configs
  for (auto &it : this->addedConfigs)
  {
    auto configData = it.second;

    this->listLayout->removeWidget(configData->widget);
    delete configData->widget;

    emit VisualRemoved(configData->name);

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
    emit VisualAdded(it.second->name);
  }
  this->deletedConfigs.clear();
}

/////////////////////////////////////////////////
void VisualConfigData::OnToggleItem(bool _checked)
{
  if (_checked)
    this->configWidget->show();
  else
    this->configWidget->hide();
}

/////////////////////////////////////////////////
void VisualConfigData::RestoreOriginalData()
{
  msgs::VisualPtr visualPtr;
  visualPtr.reset(new msgs::Visual);
  visualPtr->CopyFrom(this->originalDataMsg);

  // Update default widgets
  this->configWidget->blockSignals(true);
  this->configWidget->UpdateFromMsg(visualPtr.get());
  this->configWidget->blockSignals(false);
}
