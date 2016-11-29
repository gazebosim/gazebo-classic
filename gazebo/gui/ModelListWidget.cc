/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <functional>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <ignition/math/Angle.hh>

#include <sdf/sdf.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/gui/Conversions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ModelListWidget.hh"
#include "gazebo/gui/ModelListWidgetPrivate.hh"
#include "gazebo/gui/ModelRightMenu.hh"
#include "gazebo/gui/qtpropertybrowser/qttreepropertybrowser.h"
#include "gazebo/gui/qtpropertybrowser/qtvariantproperty.h"

#include "gazebo/rendering/Grid.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

// avoid collision from Mac OS X's ConditionalMacros.h
#ifdef __MACH__
#undef TYPE_BOOL
#endif

using namespace gazebo;
using namespace gui;

extern ModelRightMenu *g_modelRightMenu;

/////////////////////////////////////////////////
ModelListWidget::ModelListWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new ModelListWidgetPrivate)
{
  this->setObjectName("modelList");

  this->dataPtr->requestMsg = nullptr;
  this->dataPtr->propMutex = new std::mutex();
  this->dataPtr->receiveMutex = new std::mutex();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->dataPtr->modelTreeWidget = new QTreeWidget();
  this->dataPtr->modelTreeWidget->setObjectName("modelTreeWidget");
  this->dataPtr->modelTreeWidget->setColumnCount(1);
  this->dataPtr->modelTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->dataPtr->modelTreeWidget->header()->hide();
  this->dataPtr->modelTreeWidget->setFocusPolicy(Qt::NoFocus);
  this->dataPtr->modelTreeWidget->setSelectionMode(
      QAbstractItemView::ExtendedSelection);
  this->dataPtr->modelTreeWidget->setSelectionBehavior(
      QAbstractItemView::SelectRows);
  this->dataPtr->modelTreeWidget->setVerticalScrollMode(
      QAbstractItemView::ScrollPerPixel);

  connect(this->dataPtr->modelTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *,
      int)),
          this, SLOT(OnModelSelection(QTreeWidgetItem *, int)));
  connect(this->dataPtr->modelTreeWidget,
      SIGNAL(customContextMenuRequested(const QPoint &)),
      this, SLOT(OnCustomContextMenu(const QPoint &)));

  this->dataPtr->variantManager = new QtVariantPropertyManager();
  this->dataPtr->propTreeBrowser = new QtTreePropertyBrowser();
  this->dataPtr->propTreeBrowser->setObjectName("propTreeBrowser");
  this->dataPtr->propTreeBrowser->setStyleSheet(
      "QTreeView::branch:selected:active { background-color: transparent; }");
  this->dataPtr->variantFactory = new QtVariantEditorFactory();
  this->dataPtr->propTreeBrowser->setFactoryForManager(
      this->dataPtr->variantManager,
      this->dataPtr->variantFactory);
  connect(this->dataPtr->variantManager,
          SIGNAL(propertyChanged(QtProperty*)),
          this, SLOT(OnPropertyChanged(QtProperty *)));
  connect(this->dataPtr->propTreeBrowser,
          SIGNAL(currentItemChanged(QtBrowserItem*)),
          this, SLOT(OnCurrentPropertyChanged(QtBrowserItem *)));

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;

  QSplitter *splitter = new QSplitter(Qt::Vertical, this);
  splitter->addWidget(this->dataPtr->modelTreeWidget);
  splitter->addWidget(this->dataPtr->propTreeBrowser);
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

  this->ResetTree();

  this->dataPtr->connections.push_back(
      gui::Events::ConnectModelUpdate(
        std::bind(&ModelListWidget::OnModelUpdate, this,
          std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectLightUpdate(
        std::bind(&ModelListWidget::OnLightUpdate, this,
          std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      rendering::Events::ConnectCreateScene(
        std::bind(&ModelListWidget::OnCreateScene, this,
          std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      rendering::Events::ConnectRemoveScene(
        std::bind(&ModelListWidget::OnRemoveScene, this,
          std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      event::Events::ConnectSetSelectedEntity(
        std::bind(&ModelListWidget::OnSetSelectedEntity, this,
          std::placeholders::_1, std::placeholders::_2)));

  QTimer::singleShot(500, this, SLOT(Update()));
}

/////////////////////////////////////////////////
ModelListWidget::~ModelListWidget()
{
  this->dataPtr->responseSub.reset();
  this->dataPtr->requestSub.reset();
  this->dataPtr->requestPub.reset();
  this->dataPtr->modelPub.reset();
  this->dataPtr->scenePub.reset();
  this->dataPtr->physicsPub.reset();
  this->dataPtr->atmospherePub.reset();
  this->dataPtr->windPub.reset();
  this->dataPtr->lightPub.reset();
  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
  this->dataPtr->connections.clear();
  delete this->dataPtr->propMutex;
  delete this->dataPtr->receiveMutex;
  this->dataPtr->node.reset();
}

/////////////////////////////////////////////////
void ModelListWidget::OnModelSelection(QTreeWidgetItem *_item, int /*_column*/)
{
  if (_item)
  {
    std::string name = _item->data(0, Qt::UserRole).toString().toStdString();
    this->dataPtr->propTreeBrowser->clear();
    if (name == "Scene")
    {
      this->dataPtr->requestMsg = msgs::CreateRequest("scene_info",
                         this->dataPtr->selectedEntityName);
      this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);
    }
    else if (name == "Models")
    {
      this->dataPtr->modelsItem->setExpanded(
          !this->dataPtr->modelsItem->isExpanded());
    }
    else if (name == "Lights")
    {
      this->dataPtr->lightsItem->setExpanded(
          !this->dataPtr->lightsItem->isExpanded());
    }
    else if (name == "Physics")
    {
      this->dataPtr->requestMsg = msgs::CreateRequest("physics_info",
                                             this->dataPtr->selectedEntityName);
      this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);
    }
    else if (name == "Atmosphere")
    {
      this->dataPtr->requestMsg = msgs::CreateRequest("atmosphere_info",
                                             this->dataPtr->selectedEntityName);
      this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);
    }
    else if (name == "Wind")
    {
      this->dataPtr->requestMsg = msgs::CreateRequest("wind_info",
                                             this->dataPtr->selectedEntityName);
      this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);
    }
    else if (name == "Spherical Coordinates")
    {
      this->dataPtr->requestMsg = msgs::CreateRequest(
          "spherical_coordinates_info",
          this->dataPtr->selectedEntityName);
      this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);
    }
    else if (name == "GUI")
    {
      this->FillUserCamera();
      this->FillGrid();
    }
    else
    {
      this->dataPtr->propTreeBrowser->clear();
      event::Events::setSelectedEntity(name, "normal");
    }
  }
  else
    this->dataPtr->selectedEntityName.clear();
}

/////////////////////////////////////////////////
void ModelListWidget::OnSetSelectedEntity(const std::string &_name,
                                          const std::string &/*_mode*/)
{
  this->dataPtr->selectedEntityName = _name;

  this->dataPtr->propTreeBrowser->clear();
  if (!this->dataPtr->selectedEntityName.empty())
  {
    QTreeWidgetItem *mItem = this->ListItem(
      this->dataPtr->selectedEntityName,
      this->dataPtr->modelsItem);
    QTreeWidgetItem *lItem = this->ListItem(
      this->dataPtr->selectedEntityName,
      this->dataPtr->lightsItem);
    if (mItem)
    {
      if (this->dataPtr->requestPub)
      {
        if (mItem->data(3, Qt::UserRole).toString().toStdString() == "Plugin")
        {
          this->dataPtr->requestMsg = msgs::CreateRequest("model_plugin_info",
              this->dataPtr->selectedEntityName);
        }
        else
        {
          this->dataPtr->requestMsg = msgs::CreateRequest("entity_info",
           this->dataPtr->selectedEntityName);
        }
        this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);
      }
      this->dataPtr->modelTreeWidget->setCurrentItem(mItem);
      mItem->setExpanded(!mItem->isExpanded());
    }
    else if (lItem)
    {
      rendering::LightPtr light =
        gui::get_active_camera()->GetScene()->GetLight(
            this->dataPtr->selectedEntityName);

      light->FillMsg(this->dataPtr->lightMsg);
      this->dataPtr->propTreeBrowser->clear();
      this->dataPtr->fillTypes.push_back("Light");

      this->dataPtr->modelTreeWidget->setCurrentItem(lItem);
    }
  }
  else if (this->dataPtr->modelTreeWidget->currentItem())
  {
    this->dataPtr->modelTreeWidget->currentItem()->setSelected(false);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::Update()
{
  if (!this->dataPtr->fillTypes.empty())
  {
    std::lock_guard<std::mutex> lock(*this->dataPtr->propMutex);
    this->dataPtr->fillingPropertyTree = true;
    this->dataPtr->propTreeBrowser->clear();

    if (this->dataPtr->fillTypes[0] == "Model")
      this->FillPropertyTree(this->dataPtr->modelMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Link")
      this->FillPropertyTree(this->dataPtr->linkMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Joint")
      this->FillPropertyTree(this->dataPtr->jointMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Plugin")
      this->FillPropertyTree(this->dataPtr->pluginMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Scene")
      this->FillPropertyTree(this->dataPtr->sceneMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Physics")
      this->FillPropertyTree(this->dataPtr->physicsMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Atmosphere")
      this->FillPropertyTree(this->dataPtr->atmosphereMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Wind")
      this->FillPropertyTree(this->dataPtr->windMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Light")
      this->FillPropertyTree(this->dataPtr->lightMsg, nullptr);
    else if (this->dataPtr->fillTypes[0] == "Spherical Coordinates")
      this->FillPropertyTree(this->dataPtr->sphericalCoordMsg, nullptr);
    this->dataPtr->fillingPropertyTree = false;
    this->dataPtr->fillTypes.pop_front();
  }

  if (!this->dataPtr->modelTreeWidget->currentItem())
  {
    std::lock_guard<std::mutex> lock(*this->dataPtr->propMutex);
    this->dataPtr->propTreeBrowser->clear();
  }

  this->ProcessRemoveEntity();
  this->ProcessModelMsgs();
  this->ProcessLightMsgs();
  QTimer::singleShot(1000, this, SLOT(Update()));
}

/////////////////////////////////////////////////
void ModelListWidget::OnModelUpdate(const msgs::Model &_msg)
{
  std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);
  msgs::Model msg;
  msg.CopyFrom(_msg);
  this->dataPtr->modelMsgs.push_back(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::OnLightUpdate(const msgs::Light &_msg)
{
  std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);
  msgs::Light msg;
  msg.CopyFrom(_msg);
  this->dataPtr->lightMsgs.push_back(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::ProcessModelMsgs()
{
  std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);
  QFont subheaderFont;
  subheaderFont.setBold(true);

  for (auto iter = this->dataPtr->modelMsgs.begin();
       iter != this->dataPtr->modelMsgs.end(); ++iter)
  {
    std::string name = (*iter).name();

    QTreeWidgetItem *listItem = this->ListItem((*iter).name(),
                                                  this->dataPtr->modelsItem);

    if (!listItem)
    {
      if (!(*iter).has_deleted() || !(*iter).deleted())
      {
        // Create an item for the model name
        QTreeWidgetItem *topItem = new QTreeWidgetItem(
            this->dataPtr->modelsItem,
            QStringList(QString("%1").arg(QString::fromStdString(name))));

        topItem->setData(0, Qt::UserRole, QVariant((*iter).name().c_str()));
        this->dataPtr->modelTreeWidget->addTopLevelItem(topItem);

        if ((*iter).link_size() > 0)
        {
          // Create subheader for links
          QTreeWidgetItem *linkHeaderItem = new QTreeWidgetItem(topItem,
          QStringList(QString("%1").arg(QString::fromStdString("LINKS"))));
          linkHeaderItem->setFont(0, subheaderFont);
          linkHeaderItem->setFlags(Qt::NoItemFlags);
          this->dataPtr->modelTreeWidget->addTopLevelItem(linkHeaderItem);
        }

        for (int i = 0; i < (*iter).link_size(); ++i)
        {
          std::string linkName = (*iter).link(i).name();
          int index = linkName.rfind("::") + 2;
          std::string linkNameShort = linkName.substr(index,
                                                      linkName.size() - index);

          QTreeWidgetItem *linkItem = new QTreeWidgetItem(topItem,
              QStringList(QString("%1").arg(
                  QString::fromStdString(linkNameShort))));

          linkItem->setData(0, Qt::UserRole, QVariant(linkName.c_str()));
          linkItem->setData(1, Qt::UserRole, QVariant((*iter).name().c_str()));
          linkItem->setData(2, Qt::UserRole, QVariant((*iter).id()));
          linkItem->setData(3, Qt::UserRole, QVariant("Link"));
          this->dataPtr->modelTreeWidget->addTopLevelItem(linkItem);
        }

        if ((*iter).joint_size() > 0)
        {
          // Create subheader for joints
          QTreeWidgetItem *jointHeaderItem = new QTreeWidgetItem(topItem,
          QStringList(QString("%1").arg(QString::fromStdString("JOINTS"))));
          jointHeaderItem->setFont(0, subheaderFont);
          jointHeaderItem->setFlags(Qt::NoItemFlags);
          this->dataPtr->modelTreeWidget->addTopLevelItem(jointHeaderItem);
        }

        for (int i = 0; i < (*iter).joint_size(); ++i)
        {
          std::string jointName = (*iter).joint(i).name();

          int index = jointName.rfind("::") + 2;
          std::string jointNameShort = jointName.substr(
              index, jointName.size() - index);

          QTreeWidgetItem *jointItem = new QTreeWidgetItem(topItem,
              QStringList(QString("%1").arg(
                  QString::fromStdString(jointNameShort))));

          jointItem->setData(0, Qt::UserRole, QVariant(jointName.c_str()));
          jointItem->setData(3, Qt::UserRole, QVariant("Joint"));
          this->dataPtr->modelTreeWidget->addTopLevelItem(jointItem);
        }

        if ((*iter).plugin_size() > 0)
        {
          // Create subheader for plugins
          QTreeWidgetItem *pluginHeaderItem = new QTreeWidgetItem(topItem,
          QStringList(QString("%1").arg("PLUGINS")));
          pluginHeaderItem->setFont(0, subheaderFont);
          pluginHeaderItem->setFlags(Qt::NoItemFlags);
          this->dataPtr->modelTreeWidget->addTopLevelItem(pluginHeaderItem);
        }

        for (int i = 0; i < (*iter).plugin_size(); ++i)
        {
          std::string pluginName = (*iter).plugin(i).name();

          QTreeWidgetItem *pluginItem = new QTreeWidgetItem(topItem,
              QStringList(QString("%1").arg(
                  QString::fromStdString(pluginName))));

          common::URI pluginUri;
          pluginUri.SetScheme("data");

          pluginUri.Path().PushBack("world");
          pluginUri.Path().PushBack(gui::get_world());
          pluginUri.Path().PushBack("model");
          pluginUri.Path().PushBack((*iter).name());
          pluginUri.Path().PushBack("plugin");
          pluginUri.Path().PushBack(pluginName);

          pluginItem->setData(0, Qt::UserRole,
              QVariant(pluginUri.Str().c_str()));
          pluginItem->setData(3, Qt::UserRole, QVariant("Plugin"));

          this->dataPtr->modelTreeWidget->addTopLevelItem(pluginItem);
        }
      }
    }
    else
    {
      if ((*iter).has_deleted() && (*iter).deleted())
      {
        int i = this->dataPtr->modelsItem->indexOfChild(listItem);
        this->dataPtr->modelsItem->takeChild(i);
      }
      else
      {
        listItem->setText(0, (*iter).name().c_str());
        listItem->setData(1, Qt::UserRole, QVariant((*iter).name().c_str()));
      }
    }
  }
  this->dataPtr->modelMsgs.clear();
}

/////////////////////////////////////////////////
void ModelListWidget::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->dataPtr->requestMsg || _msg->id() !=
      this->dataPtr->requestMsg->id())
    return;

  if (_msg->has_type() && _msg->type() == this->dataPtr->modelMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->modelMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Model");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() ==
      this->dataPtr->linkMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->linkMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Link");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() ==
      this->dataPtr->jointMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->jointMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Joint");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() ==
    this->dataPtr->pluginMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->pluginMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Plugin");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() ==
      this->dataPtr->sceneMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->sceneMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Scene");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() ==
      this->dataPtr->physicsMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->physicsMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Physics");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() ==
      this->dataPtr->atmosphereMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->atmosphereMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Atmosphere");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() ==
      this->dataPtr->windMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->windMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Wind");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() ==
      this->dataPtr->lightMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->lightMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Light");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() &&
           _msg->type() == this->dataPtr->sphericalCoordMsg.GetTypeName())
  {
    this->dataPtr->propMutex->lock();
    this->dataPtr->sphericalCoordMsg.ParseFromString(_msg->serialized_data());
    this->dataPtr->fillTypes.push_back("Spherical Coordinates");
    this->dataPtr->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == "error")
  {
    if (_msg->response() == "nonexistent")
    {
      this->dataPtr->removeEntityList.push_back(
        this->dataPtr->selectedEntityName);
    }
  }

  delete this->dataPtr->requestMsg;
  this->dataPtr->requestMsg = nullptr;
}

/////////////////////////////////////////////////
void ModelListWidget::RemoveEntity(const std::string &_name)
{
  QTreeWidgetItem *items[2];
  items[0] = this->dataPtr->modelsItem;
  items[1] = this->dataPtr->lightsItem;

  for (int i = 0; i < 2; ++i)
  {
    QTreeWidgetItem *listItem = this->ListItem(_name, items[i]);
    if (listItem)
    {
      items[i]->takeChild(items[i]->indexOfChild(listItem));
      this->dataPtr->propTreeBrowser->clear();
      this->dataPtr->selectedEntityName.clear();
      this->dataPtr->sdfElement.reset();
      this->dataPtr->fillTypes.clear();
      return;
    }
  }
}

/////////////////////////////////////////////////
QTreeWidgetItem *ModelListWidget::ListItem(const std::string &_name,
                                              QTreeWidgetItem *_parent)
{
  QTreeWidgetItem *listItem = nullptr;

  // Find an existing element with the name from the message
  for (int i = 0; i < _parent->childCount() && !listItem; ++i)
  {
    QTreeWidgetItem *item = _parent->child(i);
    std::string listData = item->data(0, Qt::UserRole).toString().toStdString();

    if (listData == _name)
    {
      listItem = item;
      break;
    }

    for (int j = 0; j < item->childCount(); j++)
    {
      QTreeWidgetItem *childItem = item->child(j);
      listData = childItem->data(0, Qt::UserRole).toString().toStdString();
      if (listData == _name)
      {
        listItem = childItem;
        break;
      }
    }
  }

  return listItem;
}

/////////////////////////////////////////////////
void ModelListWidget::OnCustomContextMenu(const QPoint &_pt)
{
  QTreeWidgetItem *item = this->dataPtr->modelTreeWidget->itemAt(_pt);

  // Check to see if the selected item is a model
  int i = this->dataPtr->modelsItem->indexOfChild(item);
  if (i >= 0)
  {
    g_modelRightMenu->Run(item->text(0).toStdString(),
                          this->dataPtr->modelTreeWidget->mapToGlobal(_pt),
                          ModelRightMenu::EntityTypes::MODEL);
    return;
  }

  // Check to see if the selected item is a light
  i = this->dataPtr->lightsItem->indexOfChild(item);
  if (i >= 0)
  {
    g_modelRightMenu->Run(item->text(0).toStdString(),
                          this->dataPtr->modelTreeWidget->mapToGlobal(_pt),
                          ModelRightMenu::EntityTypes::LIGHT);
  }

  // Check to see if the selected item is a link
  if (item->data(3, Qt::UserRole).toString().toStdString() == "Link")
  {
    g_modelRightMenu->Run(item->data(0, Qt::UserRole).toString().toStdString(),
                          this->dataPtr->modelTreeWidget->mapToGlobal(_pt),
                          ModelRightMenu::EntityTypes::LINK);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::OnCurrentPropertyChanged(QtBrowserItem *_item)
{
  if (_item)
    this->dataPtr->selectedProperty = _item->property();
  else
    this->dataPtr->selectedProperty = nullptr;
}

/////////////////////////////////////////////////
void ModelListWidget::OnPropertyChanged(QtProperty *_item)
{
  std::unique_lock<std::mutex> lock(*this->dataPtr->propMutex, std::defer_lock);

  if (!lock.try_lock())
    return;

  if (this->dataPtr->selectedProperty != _item ||
      this->dataPtr->fillingPropertyTree)
  {
    return;
  }

  QTreeWidgetItem *currentItem =
    this->dataPtr->modelTreeWidget->currentItem();

  if (!currentItem)
    return;

  if (this->dataPtr->modelsItem->indexOfChild(currentItem) != -1 ||
      this->dataPtr->modelsItem->indexOfChild(currentItem->parent()) != -1)
    this->ModelPropertyChanged(_item);
  else if (this->dataPtr->lightsItem->indexOfChild(currentItem) != -1)
    this->LightPropertyChanged(_item);
  else if (currentItem == this->dataPtr->sceneItem)
    this->ScenePropertyChanged(_item);
  else if (currentItem == this->dataPtr->physicsItem)
    this->PhysicsPropertyChanged(_item);
  else if (currentItem == this->dataPtr->atmosphereItem)
    this->AtmospherePropertyChanged(_item);
  else if (currentItem == this->dataPtr->windItem)
    this->WindPropertyChanged(_item);
  else if (currentItem == this->dataPtr->guiItem)
    this->GUIPropertyChanged(_item);
}


/////////////////////////////////////////////////
void ModelListWidget::LightPropertyChanged(QtProperty * /*_item*/)
{
  msgs::Light msg;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == "name")
      msg.set_name(this->dataPtr->variantManager->value(
            (*iter)).toString().toStdString());
    else if ((*iter)->propertyName().toStdString() == "pose")
    {
      ignition::math::Pose3d pose;
      pose.Set(this->dataPtr->variantManager->value(
                 this->ChildItem((*iter), "x")).toDouble(),
               this->dataPtr->variantManager->value(
                 this->ChildItem((*iter), "y")).toDouble(),
               this->dataPtr->variantManager->value(
                 this->ChildItem((*iter), "z")).toDouble(),
               this->dataPtr->variantManager->value(
                 this->ChildItem((*iter), "roll")).toDouble(),
               this->dataPtr->variantManager->value(
                 this->ChildItem((*iter), "pitch")).toDouble(),
               this->dataPtr->variantManager->value(
                 this->ChildItem((*iter), "yaw")).toDouble());
      msgs::Set(msg.mutable_pose(), pose);
    }
    else if ((*iter)->propertyName().toStdString() == "range")
      msg.set_range(this->dataPtr->variantManager->value((*iter)).toDouble());
    else if ((*iter)->propertyName().toStdString() == "diffuse")
      this->FillColorMsg((*iter), msg.mutable_diffuse());
    else if ((*iter)->propertyName().toStdString() == "specular")
      this->FillColorMsg((*iter), msg.mutable_specular());
    else if ((*iter)->propertyName().toStdString() == "attenuation")
    {
      msg.set_attenuation_constant(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "constant")).toDouble());
      msg.set_attenuation_linear(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "linear")).toDouble());
      msg.set_attenuation_quadratic(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "quadratic")).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "spot light")
    {
      msg.set_spot_inner_angle(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "inner angle")).toDouble());
      msg.set_spot_outer_angle(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "outer angle")).toDouble());
      msg.set_spot_falloff(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "falloff")).toDouble());
    }
  }

  /// \TODO: Allow users to change light type
  msg.set_type(this->dataPtr->lightType);

  this->dataPtr->lightPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::GUIPropertyChanged(QtProperty *_item)
{
  // Camera
  QtProperty *cameraProperty = this->ChildItem("camera");
  if (cameraProperty && this->HasChildItem(cameraProperty, _item))
  {
    this->GUICameraPropertyChanged(_item);
    return;
  }

  // Grid
  QtProperty *gridProperty = this->ChildItem("grid");
  if (gridProperty && this->HasChildItem(gridProperty, _item))
  {
    this->GUIGridPropertyChanged(_item);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::GUICameraPropertyChanged(QtProperty *_item)
{
  QtProperty *cameraProperty = this->ChildItem("camera");
  QtProperty *cameraPoseProperty = this->ChildItem(cameraProperty, "pose");
  if (cameraPoseProperty)
  {
    std::string changedProperty = _item->propertyName().toStdString();
    if (changedProperty == "x"
      || changedProperty == "y"
      || changedProperty == "z"
      || changedProperty == "roll"
      || changedProperty == "pitch"
      || changedProperty == "yaw")
    {
      msgs::Pose poseMsg;
      this->FillPoseMsg(cameraPoseProperty, &poseMsg, poseMsg.GetDescriptor());
      rendering::UserCameraPtr cam = gui::get_active_camera();
      if (cam)
        cam->SetWorldPose(msgs::ConvertIgn(poseMsg));
    }
  }

  QtProperty *cameraClipProperty = this->ChildItem(cameraProperty, "clip");
  if (cameraPoseProperty)
  {
    std::string changedProperty = _item->propertyName().toStdString();
    rendering::UserCameraPtr cam = gui::get_active_camera();

    if (cam)
    {
      if (changedProperty == "near")
      {
        cam->SetClipDist(this->dataPtr->variantManager->value(
              this->ChildItem(cameraClipProperty, "near")).toDouble(),
            cam->FarClip());
      }
      else if (changedProperty == "far")
      {
        cam->SetClipDist(cam->NearClip(), this->dataPtr->variantManager->value(
              this->ChildItem(cameraClipProperty, "far")).toDouble());
      }
      else
      {
        gzerr << "Unable to process user camera clip property["
          << changedProperty << "]\n";
      }
    }
    else
    {
      gzerr << "Unable to get pointer to active user camera when setting clip "
        << "plane values. This should not happen.\n";
    }
  }

  QtProperty *cameraFollowProperty = this->ChildItem(cameraProperty,
                                                        "track_visual");
  if (cameraFollowProperty)
  {
    rendering::UserCameraPtr cam = gui::get_active_camera();
    if (!cam)
      return;
    std::string changedProperty = _item->propertyName().toStdString();
    if (changedProperty == "static")
    {
      cam->SetTrackIsStatic(this->dataPtr->variantManager->value(
             this->ChildItem(cameraFollowProperty, "static")).toBool());
    }
    else if (changedProperty == "use_model_frame")
    {
      cam->SetTrackUseModelFrame(this->dataPtr->variantManager->value(
             this->ChildItem(cameraFollowProperty,
                             "use_model_frame")).toBool());
    }
    else if (changedProperty == "inherit_yaw")
    {
      cam->SetTrackInheritYaw(this->dataPtr->variantManager->value(
             this->ChildItem(cameraFollowProperty, "inherit_yaw")).toBool());
    }
    else if (changedProperty == "x"
        || changedProperty == "y"
        || changedProperty == "z")
    {
      msgs::Vector3d msg;
      this->FillVector3Msg(cameraFollowProperty, &msg);
      cam->SetTrackPosition(msgs::ConvertIgn(msg));
    }
    else if (changedProperty == "min_distance")
    {
      cam->SetTrackMinDistance(this->dataPtr->variantManager->value(
             this->ChildItem(cameraFollowProperty,
               "min_distance")).toDouble());
    }
    else if (changedProperty == "max_distance")
    {
      cam->SetTrackMaxDistance(this->dataPtr->variantManager->value(
             this->ChildItem(cameraFollowProperty,
               "max_distance")).toDouble());
    }
  }
}

/////////////////////////////////////////////////
void ModelListWidget::GUIGridPropertyChanged(QtProperty *_item)
{
  if (!_item)
    return;

  auto scene = rendering::get_scene();
  if (!scene)
    return;

  // Get the main grid
  auto grid = scene->GetGrid(0);
  if (!grid)
    return;

  auto changedProperty = _item->propertyName().toStdString();
  if (changedProperty == "cell count")
  {
    grid->SetCellCount(this->dataPtr->variantManager->value(
        _item).toUInt());
  }
  else if (changedProperty == "cell size")
  {
    grid->SetCellLength(this->dataPtr->variantManager->value(
        _item).toDouble());
  }
  else if (changedProperty == "normal cell count")
  {
    grid->SetHeight(this->dataPtr->variantManager->value(
        _item).toUInt());
  }
  else if (changedProperty == "line color" || changedProperty == "Red" ||
           changedProperty == "Green" || changedProperty == "Blue" ||
           changedProperty == "Alpha")
  {
    auto lineColorItem = this->ChildItem("line color");
    grid->SetColor(gui::Conversions::Convert(
        this->dataPtr->variantManager->value(lineColorItem).value<QColor>()));
  }
  else if (changedProperty == "height offset")
  {
    grid->SetHeightOffset(
        this->dataPtr->variantManager->value(_item).toDouble());
  }
}

/////////////////////////////////////////////////
void ModelListWidget::PhysicsPropertyChanged(QtProperty * /*_item*/)
{
  msgs::Physics msg;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == "gravity")
      this->FillVector3Msg((*iter), msg.mutable_gravity());
    else if ((*iter)->propertyName().toStdString() == "magnetic field")
      this->FillVector3Msg((*iter), msg.mutable_magnetic_field());
    else if ((*iter)->propertyName().toStdString() == "enable physics")
      msg.set_enable_physics(
          this->dataPtr->variantManager->value((*iter)).toBool());
    else if ((*iter)->propertyName().toStdString() == "solver")
    {
      msg.set_iters(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "iterations")).toInt());
      msg.set_sor(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "SOR")).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "constraints")
    {
      msg.set_cfm(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "CFM")).toDouble());
      msg.set_erp(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "ERP")).toDouble());
      msg.set_contact_max_correcting_vel(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "max velocity")).toDouble());
      msg.set_contact_surface_layer(this->dataPtr->variantManager->value(
            this->ChildItem((*iter), "surface layer")).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "real time update rate")
    {
      msg.set_real_time_update_rate(
          this->dataPtr->variantManager->value((*iter)).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "max step size")
    {
      msg.set_max_step_size(
          this->dataPtr->variantManager->value((*iter)).toDouble());
    }
  }

  msg.set_type(this->dataPtr->physicsType);
  this->dataPtr->physicsPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::AtmospherePropertyChanged(QtProperty *_item)
{
  msgs::Atmosphere msg;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == "enable atmosphere")
    {
      msg.set_enable_atmosphere(this->dataPtr->variantManager->value(
            (*iter)).toBool());
    }
    else if ((*iter)->propertyName().toStdString() == "temperature")
    {
      msg.set_temperature(
          this->dataPtr->variantManager->value((*iter)).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "pressure")
    {
      msg.set_pressure(this->dataPtr->variantManager->value(
            (*iter)).toDouble());
    }
  }

  msg.set_type(this->dataPtr->atmosphereType);
  this->dataPtr->atmospherePub->Publish(msg);

  std::string changedProperty = _item->propertyName().toStdString();
  if (changedProperty == "temperature" || changedProperty == "pressure")
  {
    // Send request to retrieve new value for mass_density
    this->dataPtr->requestMsg = msgs::CreateRequest("atmosphere_info",
                                           this->dataPtr->selectedEntityName);
    this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::WindPropertyChanged(QtProperty * /*_item*/)
{
  msgs::Wind msg;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == "enable wind")
    {
      msg.set_enable_wind(
          this->dataPtr->variantManager->value((*iter)).toBool());
    }
    else if ((*iter)->propertyName().toStdString() == "linear_velocity")
    {
      this->FillVector3Msg((*iter), msg.mutable_linear_velocity());
    }
  }

  this->dataPtr->windPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::ScenePropertyChanged(QtProperty */*_item*/)
{
  msgs::Scene msg;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == "ambient")
      this->FillColorMsg((*iter), msg.mutable_ambient());
    else if ((*iter)->propertyName().toStdString() == "background")
      this->FillColorMsg((*iter), msg.mutable_background());
    else if ((*iter)->propertyName().toStdString() == "shadows")
      msg.set_shadows(this->dataPtr->variantManager->value((*iter)).toBool());
  }

  msg.set_name(gui::get_world());
  this->dataPtr->scenePub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::ModelPropertyChanged(QtProperty *_item)
{
  msgs::Model msg;

  google::protobuf::Message *fillMsg = &msg;

  QTreeWidgetItem *currentItem = this->dataPtr->modelTreeWidget->currentItem();

  // check if it's a link
  if (currentItem->data(3, Qt::UserRole).toString().toStdString() == "Link")
  {
    // this->dataPtr->modelMsg may not have been set
    // so get the model name from the current item
    msg.set_name(currentItem->data(1, Qt::UserRole).toString().toStdString());
    msg.set_id(currentItem->data(2, Qt::UserRole).toInt());

    // set link id and strip link name.
    msgs::Link *linkMsg = msg.add_link();
    linkMsg->set_id(this->dataPtr->linkMsg.id());
    std::string linkName = this->dataPtr->linkMsg.name();
    size_t index = linkName.find_last_of("::");
    if (index != std::string::npos)
      linkName = linkName.substr(index+1);
    linkMsg->set_name(linkName);
    fillMsg = linkMsg;
  }
  else
  {
    msg.set_id(this->dataPtr->modelMsg.id());
    msg.set_name(this->dataPtr->modelMsg.name());
  }

  const google::protobuf::Descriptor *descriptor = fillMsg->GetDescriptor();
  const google::protobuf::Reflection *reflection = fillMsg->GetReflection();

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if (!this->HasChildItem(*iter, _item))
      continue;

    const google::protobuf::FieldDescriptor *field =
      descriptor->FindFieldByName((*iter)->propertyName().toStdString());

    // If the message has the field, and it's another message type, then
    // recursively call FillMsg
    if (field &&
        field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE)
    {
      if (field->is_repeated())
      {
        this->FillMsg((*iter), reflection->AddMessage(fillMsg, field),
            field->message_type(), _item);
      }
      else
      {
        this->FillMsg((*iter),
            reflection->MutableMessage(fillMsg, field),
            field->message_type(), _item);
      }
    }
    else if (field)
    {
      this->FillMsgField((*iter), fillMsg, reflection, field);
    }
    else
    {
      gzerr << "Unable to process["
            << (*iter)->propertyName().toStdString() << "]\n";
    }
  }

  this->dataPtr->modelPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::FillMsgField(QtProperty *_item,
    google::protobuf::Message *_message,
    const google::protobuf::Reflection *_reflection,
    const google::protobuf::FieldDescriptor *_field)
{
  if (_field->type() == google::protobuf::FieldDescriptor::TYPE_INT32)
    _reflection->SetInt32(_message, _field,
        this->dataPtr->variantManager->value(_item).toInt());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_DOUBLE)
    _reflection->SetDouble(_message, _field,
        this->dataPtr->variantManager->value(_item).toDouble());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_FLOAT)
    _reflection->SetFloat(_message, _field,
        this->dataPtr->variantManager->value(_item).toDouble());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_BOOL)
    _reflection->SetBool(_message, _field,
        this->dataPtr->variantManager->value(_item).toBool());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_STRING)
    _reflection->SetString(_message, _field,
        this->dataPtr->variantManager->value(_item).toString().toStdString());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_UINT32)
    _reflection->SetUInt32(_message, _field,
        this->dataPtr->variantManager->value(_item).toUInt());
  else
    gzerr << "Unable to fill message field[" << _field->type() << "]\n";
}

/////////////////////////////////////////////////
void ModelListWidget::FillColorMsg(QtProperty *_item, msgs::Color *_msg)
{
  _msg->set_r(this->dataPtr->variantManager->value(
      this->ChildItem(_item, "Red")).toDouble() / 255.0);
  _msg->set_g(this->dataPtr->variantManager->value(
      this->ChildItem(_item, "Green")).toDouble() / 255.0);
  _msg->set_b(this->dataPtr->variantManager->value(
      this->ChildItem(_item, "Blue")).toDouble() / 255.0);
  _msg->set_a(this->dataPtr->variantManager->value(
      this->ChildItem(_item, "Alpha")).toDouble() / 255.0);
}

/////////////////////////////////////////////////
void ModelListWidget::FillVector3Msg(QtProperty *_item, msgs::Vector3d *_msg)
{
  _msg->set_x(this->dataPtr->variantManager->value(
      this->ChildItem(_item, "x")).toDouble());
  _msg->set_y(this->dataPtr->variantManager->value(
      this->ChildItem(_item, "y")).toDouble());
  _msg->set_z(this->dataPtr->variantManager->value(
      this->ChildItem(_item, "z")).toDouble());
}

/////////////////////////////////////////////////
void ModelListWidget::FillGeometryMsg(QtProperty *_item,
    google::protobuf::Message *_message,
    const google::protobuf::Descriptor *_descriptor,
    QtProperty * /*_changedItem*/)
{
  QtProperty *typeProperty = this->ChildItem(_item, "type");

  std::string type = typeProperty->valueText().toStdString();
  const google::protobuf::Reflection *reflection = _message->GetReflection();
  reflection->SetEnum(_message, _descriptor->FindFieldByName("type"),
      _descriptor->FindEnumValueByName(type));

  // make sure type content is lowercase
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  const google::protobuf::FieldDescriptor *field =
    _descriptor->FindFieldByName(type);
  google::protobuf::Message *message =
    _message->GetReflection()->MutableMessage(_message, field);

  if (type == "box")
  {
    QtProperty *sizeProperty = this->ChildItem(_item, "size");
    msgs::BoxGeom *boxMsg = (msgs::BoxGeom*)(message);
    double xValue = this->dataPtr->variantManager->value(
        this->ChildItem(sizeProperty, "x")).toDouble();
    double yValue = this->dataPtr->variantManager->value(
        this->ChildItem(sizeProperty, "y")).toDouble();
    double zValue = this->dataPtr->variantManager->value(
        this->ChildItem(sizeProperty, "z")).toDouble();

    boxMsg->mutable_size()->set_x(xValue);
    boxMsg->mutable_size()->set_y(yValue);
    boxMsg->mutable_size()->set_z(zValue);
  }
  else if (type == "sphere")
  {
    QtProperty *radiusProperty = this->ChildItem(_item, "radius");
    msgs::SphereGeom *sphereMsg = (msgs::SphereGeom*)(message);

    sphereMsg->set_radius(
        this->dataPtr->variantManager->value(radiusProperty).toDouble());
  }
  else if (type == "cylinder")
  {
    QtProperty *radiusProperty = this->ChildItem(_item, "radius");
    QtProperty *lengthProperty = this->ChildItem(_item, "length");

    msgs::CylinderGeom *cylinderMsg = (msgs::CylinderGeom*)(message);
    cylinderMsg->set_radius(
        this->dataPtr->variantManager->value(radiusProperty).toDouble());
    cylinderMsg->set_length(
        this->dataPtr->variantManager->value(lengthProperty).toDouble());
  }
  else if (type == "plane")
  {
    QtProperty *normalProperty = this->ChildItem(_item, "normal");
    msgs::PlaneGeom *planeMessage = (msgs::PlaneGeom*)(message);

    double xValue = this->dataPtr->variantManager->value(
        this->ChildItem(normalProperty, "x")).toDouble();
    double yValue = this->dataPtr->variantManager->value(
        this->ChildItem(normalProperty, "y")).toDouble();
    double zValue = this->dataPtr->variantManager->value(
        this->ChildItem(normalProperty, "z")).toDouble();

    planeMessage->mutable_normal()->set_x(xValue);
    planeMessage->mutable_normal()->set_y(yValue);
    planeMessage->mutable_normal()->set_z(zValue);
  }
  else if (type == "image")
  {
    QtProperty *fileProp = this->ChildItem(_item, "filename");
    QtProperty *scaleProp = this->ChildItem(_item, "scale");
    QtProperty *heightProp = this->ChildItem(_item, "height");
    QtProperty *thresholdProp = this->ChildItem(_item, "threshold");
    QtProperty *granularityProp = this->ChildItem(_item, "granularity");

    msgs::ImageGeom *imageMessage = (msgs::ImageGeom*)(message);
    imageMessage->set_uri(
       this->dataPtr->variantManager->value(fileProp).toString().toStdString());
    imageMessage->set_scale(
        this->dataPtr->variantManager->value(scaleProp).toDouble());
    imageMessage->set_height(
        this->dataPtr->variantManager->value(heightProp).toDouble());
    imageMessage->set_threshold(
        this->dataPtr->variantManager->value(thresholdProp).toInt());
    imageMessage->set_granularity(
        this->dataPtr->variantManager->value(granularityProp).toInt());
  }
  else if (type == "heightmap")
  {
    QtProperty *sizeProp = this->ChildItem(_item, "size");
    QtProperty *offsetProp = this->ChildItem(_item, "offset");
    QtProperty *fileProp = this->ChildItem(_item, "filename");

    double px, py, pz;
    msgs::HeightmapGeom *heightmapMessage = (msgs::HeightmapGeom*)(message);

    msgs::Set(heightmapMessage->mutable_image(),
        common::Image(this->dataPtr->variantManager->value(
            fileProp).toString().toStdString()));

    px = this->dataPtr->variantManager->value(
         this->ChildItem(sizeProp, "x")).toDouble();
    py = this->dataPtr->variantManager->value(
         this->ChildItem(sizeProp, "y")).toDouble();
    pz = this->dataPtr->variantManager->value(
         this->ChildItem(sizeProp, "z")).toDouble();

    heightmapMessage->mutable_size()->set_x(px);
    heightmapMessage->mutable_size()->set_y(py);
    heightmapMessage->mutable_size()->set_z(pz);

    px = this->dataPtr->variantManager->value(
         this->ChildItem(offsetProp, "x")).toDouble();
    py = this->dataPtr->variantManager->value(
         this->ChildItem(offsetProp, "y")).toDouble();
    pz = this->dataPtr->variantManager->value(
         this->ChildItem(offsetProp, "z")).toDouble();

    heightmapMessage->mutable_origin()->set_x(px);
    heightmapMessage->mutable_origin()->set_y(py);
    heightmapMessage->mutable_origin()->set_z(pz);
  }
  else if (type == "mesh")
  {
    QtProperty *sizeProp = this->ChildItem(_item, "scale");
    QtProperty *fileProp = this->ChildItem(_item, "filename");

    double px, py, pz;
    msgs::MeshGeom *meshMessage = (msgs::MeshGeom*)(message);
    meshMessage->set_filename(this->dataPtr->variantManager->value(
          fileProp).toString().toStdString());

    px = this->dataPtr->variantManager->value(
         this->ChildItem(sizeProp, "x")).toDouble();
    py = this->dataPtr->variantManager->value(
         this->ChildItem(sizeProp, "y")).toDouble();
    pz = this->dataPtr->variantManager->value(
         this->ChildItem(sizeProp, "z")).toDouble();

    meshMessage->mutable_scale()->set_x(px);
    meshMessage->mutable_scale()->set_y(py);
    meshMessage->mutable_scale()->set_z(pz);
  }
  else
    gzerr << "Unknown geom type[" << type << "]\n";
}

/////////////////////////////////////////////////
void ModelListWidget::FillPoseMsg(QtProperty *_item,
    google::protobuf::Message *_message,
    const google::protobuf::Descriptor *_descriptor)
{
  const google::protobuf::Descriptor *posDescriptor;
  const google::protobuf::FieldDescriptor *posField;
  google::protobuf::Message *posMessage;
  const google::protobuf::Reflection *posReflection;

  const google::protobuf::Descriptor *orientDescriptor;
  const google::protobuf::FieldDescriptor *orientField;
  google::protobuf::Message *orientMessage;
  const google::protobuf::Reflection *orientReflection;

  posField = _descriptor->FindFieldByName("position");
  posDescriptor = posField->message_type();
  posMessage = _message->GetReflection()->MutableMessage(_message, posField);
  posReflection = posMessage->GetReflection();

  orientField = _descriptor->FindFieldByName("orientation");
  orientDescriptor = orientField->message_type();
  orientMessage = _message->GetReflection()->MutableMessage(
      _message, orientField);
  orientReflection = orientMessage->GetReflection();

  this->FillMsgField(this->ChildItem(_item, "x"), posMessage, posReflection,
      posDescriptor->FindFieldByName("x"));
  this->FillMsgField(this->ChildItem(_item, "y"), posMessage, posReflection,
      posDescriptor->FindFieldByName("y"));
  this->FillMsgField(this->ChildItem(_item, "z"), posMessage, posReflection,
      posDescriptor->FindFieldByName("z"));

  double roll, pitch, yaw;
  roll = this->dataPtr->variantManager->value(
      this->ChildItem(_item, "roll")).toDouble();
  pitch = this->dataPtr->variantManager->value(
      this->ChildItem(_item, "pitch")).toDouble();
  yaw = this->dataPtr->variantManager->value(
      this->ChildItem(_item, "yaw")).toDouble();
  ignition::math::Quaterniond q(roll, pitch, yaw);

  orientReflection->SetDouble(
      orientMessage,
      orientDescriptor->FindFieldByName("x"),
      q.X());
  orientReflection->SetDouble(
      orientMessage,
      orientDescriptor->FindFieldByName("y"),
      q.Y());
  orientReflection->SetDouble(
      orientMessage,
      orientDescriptor->FindFieldByName("z"),
      q.Z());
  orientReflection->SetDouble(
      orientMessage,
      orientDescriptor->FindFieldByName("w"),
      q.W());
}

/////////////////////////////////////////////////
void ModelListWidget::FillMsg(QtProperty *_item,
    google::protobuf::Message *_message,
    const google::protobuf::Descriptor *_descriptor,
    QtProperty *_changedItem)
{
  if (!_item)
    return;

  if (_item->propertyName().toStdString() == "link")
  {
    QtProperty *nameItem = this->ChildItem(_item, "name");
    QtVariantProperty *idItem =
        dynamic_cast<QtVariantProperty *>(this->ChildItem(_item, "id"));
    ((msgs::Link*)(_message))->set_name(nameItem->valueText().toStdString());
    ((msgs::Link*)(_message))->set_id(idItem->value().toInt());
  }
  else if (_item->propertyName().toStdString() == "collision")
  {
    QtProperty *nameItem = this->ChildItem(_item, "name");
    QtVariantProperty *idItem =
        dynamic_cast<QtVariantProperty *>(this->ChildItem(_item, "id"));
    ((msgs::Collision*)_message)->set_name(nameItem->valueText().toStdString());
    ((msgs::Collision*)(_message))->set_id(idItem->value().toInt());
  }

  if (_item->propertyName().toStdString() == "geometry" &&
      this->HasChildItem(_item, _changedItem))
  {
    this->FillGeometryMsg(_item, _message, _descriptor, _changedItem);
  }
  else if (_item->propertyName().toStdString() == "pose")
  {
    if (this->HasChildItem(_item, _changedItem))
    {
      this->FillPoseMsg(_item, _message, _descriptor);
    }
  }
  else
  {
    const google::protobuf::Reflection *reflection = _message->GetReflection();

    QList<QtProperty*> properties = _item->subProperties();
    for (QList<QtProperty*>::iterator iter = properties.begin();
        iter != properties.end(); ++iter)
    {
      if (!this->HasChildItem(*iter, _changedItem))
        continue;
      const google::protobuf::FieldDescriptor *field =
        _descriptor->FindFieldByName((*iter)->propertyName().toStdString());

      // If the message has the field, and it's another message type, then
      // recursively call FillMsg
      if (field &&
          field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE)
      {
        if (field->is_repeated())
        {
          this->FillMsg((*iter), reflection->AddMessage(_message, field),
              field->message_type(), _changedItem);
        }
        else
        {
          this->FillMsg((*iter),
              reflection->MutableMessage(_message, field),
              field->message_type(), _changedItem);
        }
      }
      else if (field)
      {
        this->FillMsgField((*iter), _message, reflection, field);
      }
      else
      {
        gzerr << "Unable to process["
          << (*iter)->propertyName().toStdString() << "]\n";
      }
    }
  }
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::PopChildItem(QList<QtProperty*> &_list,
    const std::string &_name)
{
  for (QList<QtProperty*>::iterator iter = _list.begin();
      iter != _list.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == _name)
    {
      iter = _list.erase(iter);
      return (*iter);
    }
  }

  return nullptr;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::ParentItemValue(const std::string &_name)
{
  QtProperty *result = nullptr;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
      iter != properties.end(); ++iter)
  {
    if ((*iter)->valueText().toStdString() == _name)
      return nullptr;
    else if ((result = this->ParentItemValue(*iter, _name)) != nullptr)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::ParentItemValue(QtProperty *_item,
                                             const std::string &_name)
{
  if (!_item)
    return nullptr;

  QtProperty *result = nullptr;

  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((*iter)->valueText().toStdString() == _name)
    {
      result = _item;
      break;
    }
    else if ((result = this->ParentItemValue(*iter, _name)) != nullptr)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::ParentItem(const std::string &_name)
{
  QtProperty *result = nullptr;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
      iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == _name)
      return nullptr;
    else if ((result = this->ParentItem(*iter, _name)) != nullptr)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::ParentItem(QtProperty *_item,
                                        const std::string &_name)
{
  if (!_item)
    return nullptr;

  QtProperty *result = nullptr;

  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == _name)
    {
      result = _item;
      break;
    }
    else if ((result = this->ParentItem(*iter, _name)) != nullptr)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
bool ModelListWidget::HasChildItem(QtProperty *_parent, QtProperty *_child)
{
  if (!_parent)
    return false;
  if (_parent == _child)
    return true;

  bool result = false;
  QList<QtProperty*> subProperties = _parent->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((result = this->HasChildItem(*iter, _child)))
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::ChildItemValue(const std::string &_name)
{
  QtProperty *result = nullptr;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
      iter != properties.end(); ++iter)
  {
    if ((result = this->ChildItemValue(*iter, _name)) != nullptr)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::ChildItemValue(QtProperty *_item,
                                          const std::string &_name)
{
  if (!_item)
    return nullptr;
  if (_item->valueText().toStdString() == _name)
    return _item;

  QtProperty *result = nullptr;
  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((result = this->ChildItem(*iter, _name)) != nullptr)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::ChildItem(const std::string &_name)
{
  QtProperty *result = nullptr;

  QList<QtProperty*> properties = this->dataPtr->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
      iter != properties.end(); ++iter)
  {
    if ((result = this->ChildItem(*iter, _name)) != nullptr)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::ChildItem(QtProperty *_item,
                                       const std::string &_name)
{
  if (!_item)
    return nullptr;
  if (_item->propertyName().toStdString() == _name)
    return _item;

  QtProperty *result = nullptr;
  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((result = this->ChildItem(*iter, _name)) != nullptr)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::SphericalCoordinates &_msg,
                                       QtProperty * /*_parent*/)
{
  QtVariantProperty *item = nullptr;

  item = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::enumTypeId(), tr("Surface Model"));
  QStringList types;

  const google::protobuf::EnumDescriptor *surfaceModelEnum =
    _msg.GetDescriptor()->FindEnumTypeByName("SurfaceModel");

  if (!surfaceModelEnum)
  {
    gzerr << "Unable to get SurfaceModel enum descriptor from "
      << "SphericalCoordinates message. msgs::SphericalCoordinates "
      << "has probably changed\n";
    types << "invalid";
  }
  else
  {
    for (int i = 0; i < surfaceModelEnum->value_count(); ++i)
      types << surfaceModelEnum->value(i)->name().c_str();
  }

  item->setAttribute("enumNames", types);
  item->setValue(0);
  this->dataPtr->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("Latitude"));
  item->setValue(_msg.latitude_deg());
  this->dataPtr->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("Longitude"));
  item->setValue(_msg.longitude_deg());
  this->dataPtr->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("Elevation"));
  item->setValue(_msg.elevation());
  this->dataPtr->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("Heading"));
  item->setValue(_msg.heading_deg());
  this->dataPtr->propTreeBrowser->addProperty(item);
  item->setEnabled(false);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Joint &_msg,
                                       QtProperty * /*_parent*/)
{
  QtProperty *topItem = nullptr;
  QtVariantProperty *item = nullptr;

  // joint name
  item = this->dataPtr->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  this->dataPtr->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  // joint type
  if (_msg.has_type())
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::String,
                                             tr("type"));
    std::string jointType = msgs::ConvertJointType(_msg.type());
    item->setValue(jointType.c_str());
    this->dataPtr->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  // parent link
  if (_msg.has_parent())
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::String,
                                               tr("parent link"));
    item->setValue(_msg.parent().c_str());
    this->dataPtr->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  // child link
  if (_msg.has_child())
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::String,
                                               tr("child link"));
    item->setValue(_msg.child().c_str());
    this->dataPtr->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  // Pose value
  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  this->dataPtr->propTreeBrowser->addProperty(topItem);
  topItem->setEnabled(false);

  this->FillPoseProperty(_msg.pose(), topItem);

  // Angle
  for (int i = 0; i < _msg.angle_size(); ++i)
  {
    std::string angleName = "angle_" + std::to_string(i);
    item = this->dataPtr->variantManager->addProperty(QVariant::String,
                                             QString::fromStdString(angleName));
    item->setValue(_msg.angle(i));
    this->dataPtr->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  // Add joint axes if present
  for (int i = 0; i < 2; ++i)
  {
    const msgs::Axis *axis = nullptr;
    std::string axisName;

    if (i == 0 && _msg.has_axis1())
    {
      axis = &(_msg.axis1());
      axisName = "axis1";
    }
    else if (i == 1 && _msg.has_axis2())
    {
      axis = &(_msg.axis2());
      axisName = "axis2";
    }

    if (axis)
    {
      // Axis shape value
      topItem = this->dataPtr->variantManager->addProperty(
          QtVariantPropertyManager::groupTypeId(), tr(axisName.c_str()));
      this->dataPtr->propTreeBrowser->addProperty(topItem);
      topItem->setEnabled(false);

      /// XYZ of the axis
      QtProperty *xyzItem = this->dataPtr->variantManager->addProperty(
          QtVariantPropertyManager::groupTypeId(), tr("xyz"));
      topItem->addSubProperty(xyzItem);
      xyzItem->setEnabled(false);
      this->FillVector3dProperty(axis->xyz(), xyzItem);

      // lower limit
      item = this->dataPtr->variantManager->addProperty(QVariant::Double,
          tr("lower"));
      item->setValue(axis->limit_lower());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // upper limit
      item = this->dataPtr->variantManager->addProperty(QVariant::Double,
          tr("upper"));
      item->setValue(axis->limit_upper());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // limit effort
      item = this->dataPtr->variantManager->addProperty(QVariant::Double,
          tr("effort"));
      item->setValue(axis->limit_effort());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // limit velocity
      item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                               tr("velocity"));
      item->setValue(axis->limit_velocity());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // damping
      item = this->dataPtr->variantManager->addProperty(QVariant::Double,
          tr("damping"));
      item->setValue(axis->damping());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // friction
      item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                               tr("friction"));
      item->setValue(axis->friction());
      topItem->addSubProperty(item);
      item->setEnabled(false);
    }
  }

  // gearbox
  if (_msg.has_gearbox())
  {
    msgs::Joint::Gearbox gearboxMsg = _msg.gearbox();
    if (gearboxMsg.has_gearbox_reference_body())
    {
      item = this->dataPtr->variantManager->addProperty(QVariant::String,
          tr("gearbox_reference_body"));
      item->setValue(gearboxMsg.gearbox_reference_body().c_str());
      this->dataPtr->propTreeBrowser->addProperty(item);
      item->setEnabled(false);
    }
    if (gearboxMsg.has_gearbox_ratio())
    {
      item = this->dataPtr->variantManager->addProperty(QVariant::Double,
          tr("gearbox_ratio"));
      item->setValue(gearboxMsg.gearbox_ratio());
      this->dataPtr->propTreeBrowser->addProperty(item);
      item->setEnabled(false);
    }
  }

  // screw
  if (_msg.has_screw())
  {
    msgs::Joint::Screw screwMsg = _msg.screw();
    if (screwMsg.has_thread_pitch())
    {
      item = this->dataPtr->variantManager->addProperty(QVariant::Double,
          tr("thread_pitch"));
      item->setValue(screwMsg.thread_pitch());
      this->dataPtr->propTreeBrowser->addProperty(item);
      item->setEnabled(false);
    }
  }
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Link &_msg,
                                       QtProperty *_parent)
{
  QtProperty *topItem = nullptr;
  QtProperty *inertialItem = nullptr;
  QtProperty *windItem = nullptr;
  QtVariantProperty *item = nullptr;

  // id, store it but but make it hidden
  QtBrowserItem *browserItem = nullptr;
  item = this->dataPtr->variantManager->addProperty(QVariant::String, tr("id"));
  item->setValue(_msg.id());
  this->AddProperty(item, _parent);
  browserItem = this->dataPtr->propTreeBrowser->items(item)[0];
  this->dataPtr->propTreeBrowser->setItemVisible(browserItem, false);

  // name
  item = this->dataPtr->variantManager->addProperty(QVariant::String,
      tr("name"));
  item->setValue(_msg.name().c_str());
  this->AddProperty(item, _parent);
  // TODO: setting link name currently causes problems
  item->setEnabled(false);

  // Self-collide
  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
    tr("self_collide"));
  if (_msg.has_self_collide())
    item->setValue(_msg.self_collide());
  else
    item->setValue(true);
  this->AddProperty(item, _parent);
  item->setEnabled(false);

  // gravity
  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
      tr("gravity"));
  if (_msg.has_gravity())
    item->setValue(_msg.gravity());
  else
    item->setValue(true);
  this->AddProperty(item, _parent);

  // kinematic
  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
      tr("kinematic"));
  if (_msg.has_kinematic())
    item->setValue(_msg.kinematic());
  else
    item->setValue(false);
  this->AddProperty(item, _parent);

  // canonical
  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
      tr("canonical"));
  if (_msg.has_canonical())
    item->setValue(_msg.canonical());
  else
    item->setValue(false);
  this->AddProperty(item, _parent);
  item->setEnabled(false);

  // wind
  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
      tr("enable_wind"));
  if (_msg.has_enable_wind())
    item->setValue(_msg.enable_wind());
  else
    item->setValue(true);
  this->AddProperty(item, _parent);

  // pose
  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  this->AddProperty(topItem, _parent);

  this->FillPoseProperty(_msg.pose(), topItem);
  if (_msg.has_canonical() && _msg.canonical())
    topItem->setEnabled(false);

  // Inertial
  inertialItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("inertial"));
  this->AddProperty(inertialItem, _parent);

  // TODO: disable setting inertial properties until there are tests
  // in place to verify the functionality
  inertialItem->setEnabled(false);

  // Inertial::Mass
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("mass"));
  if (_msg.inertial().has_mass())
    item->setValue(_msg.inertial().mass());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::ixx
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("ixx"));
  if (_msg.inertial().has_ixx())
    item->setValue(_msg.inertial().ixx());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::ixy
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("ixy"));
  if (_msg.inertial().has_ixy())
    item->setValue(_msg.inertial().ixy());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::ixz
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("ixz"));
  if (_msg.inertial().has_ixz())
    item->setValue(_msg.inertial().ixz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::iyy
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("iyy"));
  if (_msg.inertial().has_iyy())
    item->setValue(_msg.inertial().iyy());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::iyz
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("iyz"));
  if (_msg.inertial().has_iyz())
    item->setValue(_msg.inertial().iyz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::izz
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("izz"));
  if (_msg.inertial().has_izz())
    item->setValue(_msg.inertial().izz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  inertialItem->addSubProperty(topItem);
  this->FillPoseProperty(_msg.inertial().pose(), topItem);

  // Wind
  windItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("wind"));
  this->AddProperty(windItem, _parent);
  windItem->setEnabled(false);
  if (_msg.has_wind())
    this->FillVector3dProperty(_msg.wind(), windItem);
  else
  {
    msgs::Vector3d xyz;
    xyz.set_x(0);
    xyz.set_y(0);
    xyz.set_z(0);
    this->FillVector3dProperty(xyz, windItem);
  }

  for (int i = 0; i < _msg.collision_size(); i++)
  {
    QtVariantProperty *prop;
    prop = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), tr("collision"));
    prop->setToolTip(tr(_msg.collision(i).name().c_str()));
    this->AddProperty(prop, _parent);

    this->FillPropertyTree(_msg.collision(i), prop);

    // TODO: disable setting collision properties until there are tests
    // in place to verify the functionality
    prop->setEnabled(false);
  }

  for (int i = 0; i < _msg.visual_size(); i++)
  {
    QtVariantProperty *prop;
    prop = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), tr("visual"));
    prop->setToolTip(tr(_msg.visual(i).name().c_str()));
    this->AddProperty(prop, _parent);

    this->FillPropertyTree(_msg.visual(i), prop);

    // TODO: disable setting visual properties until there are tests
    // in place to verify the functionality
    prop->setEnabled(false);
  }

  for (int i = 0; i < _msg.sensor_size(); i++)
  {
    QtVariantProperty *prop;
    prop = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), "sensor");
    prop->setToolTip(tr(_msg.sensor(i).name().c_str()));
    this->AddProperty(prop, _parent);

    // this->FillPropertyTree(_msg.sensor(i), prop);
  }

  // battery
  for (int i = 0; i < _msg.battery_size(); ++i)
  {
    QtVariantProperty *batteryItem;
    batteryItem = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), tr("battery"));
    batteryItem->setToolTip(tr(_msg.battery(i).name().c_str()));
    this->AddProperty(batteryItem, _parent);
    batteryItem->setEnabled(false);

    item = this->dataPtr->variantManager->addProperty(QVariant::String,
        tr("name"));
    item->setValue(_msg.battery(i).name().c_str());
    batteryItem->addSubProperty(item);

    // Battery::Voltage
    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
        tr("voltage"));
    item->setValue(_msg.battery(i).voltage());
    batteryItem->addSubProperty(item);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Collision &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
  {
    gzwarn << "Null QtProperty parent, not adding collision elements."
           << " This should never happen." << std::endl;
    return;
  }

  QtProperty *topItem = nullptr;
  QtVariantProperty *item = nullptr;

  // id, store it but but make it hidden
  QtBrowserItem *browserItem = nullptr;
  item = this->dataPtr->variantManager->addProperty(QVariant::String, tr("id"));
  item->setValue(_msg.id());
  _parent->addSubProperty(item);
  browserItem = this->dataPtr->propTreeBrowser->items(item)[0];
  this->dataPtr->propTreeBrowser->setItemVisible(browserItem, false);

  // name
  item = this->dataPtr->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  _parent->addSubProperty(item);

  // Laser Retro value
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("laser_retro"));
  if (_msg.has_laser_retro())
    item->setValue(_msg.laser_retro());
  else
    item->setValue(0.0);
  _parent->addSubProperty(item);

  // Pose value
  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("pose"));
  _parent->addSubProperty(topItem);
  this->FillPoseProperty(_msg.pose(), topItem);

  // Geometry shape value
  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("geometry"));
  _parent->addSubProperty(topItem);
  this->FillPropertyTree(_msg.geometry(), topItem);

  // Surface value
  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("surface"));
  _parent->addSubProperty(topItem);
  this->FillPropertyTree(_msg.surface(), topItem);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Surface &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
  {
    gzwarn << "Null QtProperty parent, not adding surface elements."
           << " This should never happen." << std::endl;
    return;
  }

  QtProperty *frictionItem = nullptr;
  QtProperty *torsionalItem = nullptr;
  QtProperty *odeItem = nullptr;
  QtVariantProperty *item = nullptr;

  // Restituion Coefficient
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("restitution_coefficient"));
  item->setValue(_msg.restitution_coefficient());
  _parent->addSubProperty(item);

  // Bounce Threshold
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("bounce_threshold"));
  item->setValue(_msg.bounce_threshold());
  _parent->addSubProperty(item);

  // Soft CFM
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("soft_cfm"));
  item->setValue(_msg.soft_cfm());
  _parent->addSubProperty(item);

  // Soft ERP
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("soft_erp"));
  item->setValue(_msg.soft_erp());
  _parent->addSubProperty(item);

  // KP
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("kp"));
  item->setValue(_msg.kp());
  _parent->addSubProperty(item);

  // KD
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("kd"));
  item->setValue(_msg.kd());
  _parent->addSubProperty(item);

  // max vel
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("max_vel"));
  item->setValue(_msg.max_vel());
  _parent->addSubProperty(item);

  // min depth
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("min_depth"));
  item->setValue(_msg.min_depth());
  _parent->addSubProperty(item);

  // Friction
  frictionItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("friction"));
  _parent->addSubProperty(frictionItem);

  // Mu
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("mu"));
  item->setValue(_msg.friction().mu());
  frictionItem->addSubProperty(item);

  // Mu2
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("mu2"));
  item->setValue(_msg.friction().mu2());
  frictionItem->addSubProperty(item);

  // slip1
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("slip1"));
  item->setValue(_msg.friction().slip1());
  frictionItem->addSubProperty(item);

  // slip2
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("slip2"));
  item->setValue(_msg.friction().slip2());
  frictionItem->addSubProperty(item);

  // Fdir1
  QtProperty *fdirItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("fdir1"));
    frictionItem->addSubProperty(fdirItem);
    this->FillVector3dProperty(_msg.friction().fdir1(), fdirItem);

  // Torsional
  torsionalItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("torsional"));
  frictionItem->addSubProperty(torsionalItem);

  // Coefficient
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("coefficient"));
  item->setValue(_msg.friction().torsional().coefficient());
  torsionalItem->addSubProperty(item);

  // Use patch radius
  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
                                           tr("use_patch_radius"));
  item->setValue(_msg.friction().torsional().use_patch_radius());
  torsionalItem->addSubProperty(item);

  // Patch radius
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("patch_radius"));
  item->setValue(_msg.friction().torsional().patch_radius());
  torsionalItem->addSubProperty(item);

  // Surface radius
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("surface_radius"));
  item->setValue(_msg.friction().torsional().surface_radius());
  torsionalItem->addSubProperty(item);

  // ODE
  odeItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("ode"));
  torsionalItem->addSubProperty(odeItem);

  // slip torsional
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("slip"));
  item->setValue(_msg.friction().torsional().ode().slip());
  odeItem->addSubProperty(item);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Geometry &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
  {
    gzwarn << "Null QtProperty parent, not adding geometry elements."
           << " This should never happen." << std::endl;
    return;
  }

  QtVariantProperty *item = nullptr;

  item = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::enumTypeId(), tr("type"));
  QStringList types;
  types << "BOX" << "SPHERE" << "CYLINDER" << "PLANE" << "MESH" << "IMAGE"
        << "HEIGHTMAP";
  item->setAttribute("enumNames", types);
  _parent->addSubProperty(item);

  if (_msg.type() == msgs::Geometry::BOX)
  {
    item->setValue(0);
    QtProperty *sizeItem = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("size"));
    _parent->addSubProperty(sizeItem);
    this->FillVector3dProperty(_msg.box().size(), sizeItem);
  }
  else if (_msg.type() == msgs::Geometry::SPHERE)
  {
    item->setValue(1);

    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
        tr("radius"));
    item->setValue(_msg.sphere().radius());
    _parent->addSubProperty(item);
  }
  else if (_msg.type() == msgs::Geometry::CYLINDER)
  {
    item->setValue(2);
    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
        tr("radius"));
    item->setValue(_msg.cylinder().radius());
    _parent->addSubProperty(item);

    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
        tr("length"));
    item->setValue(_msg.cylinder().length());
    _parent->addSubProperty(item);
  }
  else if (_msg.type() == msgs::Geometry::PLANE)
  {
    item->setValue(3);
    QtProperty *normalItem = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("normal"));
    _parent->addSubProperty(normalItem);
    this->FillVector3dProperty(_msg.plane().normal(), normalItem);
  }
  else if (_msg.type() == msgs::Geometry::MESH)
  {
    item->setValue(4);

    item = this->dataPtr->variantManager->addProperty(QVariant::String,
        tr("filename"));
    item->setValue(_msg.mesh().filename().c_str());
    _parent->addSubProperty(item);

    QtProperty *sizeItem = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("scale"));
    _parent->addSubProperty(sizeItem);
    this->FillVector3dProperty(_msg.mesh().scale(), sizeItem);
  }
  else if (_msg.type() == msgs::Geometry::IMAGE)
  {
    item->setValue(5);

    item = this->dataPtr->variantManager->addProperty(QVariant::String,
        tr("uri"));
    item->setValue(_msg.image().uri().c_str());
    _parent->addSubProperty(item);

    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
        tr("scale"));
    item->setValue(_msg.image().scale());
    _parent->addSubProperty(item);

    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
        tr("height"));
    item->setValue(_msg.image().height());
    _parent->addSubProperty(item);

    item = this->dataPtr->variantManager->addProperty(QVariant::Int,
        tr("threshold"));
    item->setValue(_msg.image().threshold());
    _parent->addSubProperty(item);

    item = this->dataPtr->variantManager->addProperty(QVariant::Int,
        tr("granularity"));
    item->setValue(_msg.image().granularity());
    _parent->addSubProperty(item);
  }
  else if (_msg.type() == msgs::Geometry::HEIGHTMAP)
  {
    item->setValue(6);

    item = this->dataPtr->variantManager->addProperty(QVariant::String,
        tr("uri"));
    item->setValue(_msg.image().uri().c_str());
    _parent->addSubProperty(item);

    QtProperty *sizeItem = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("size"));
    _parent->addSubProperty(sizeItem);
    this->FillVector3dProperty(_msg.heightmap().size(), sizeItem);

    sizeItem = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("offset"));
    _parent->addSubProperty(sizeItem);
    this->FillVector3dProperty(_msg.heightmap().origin(), sizeItem);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Visual &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
  {
    gzwarn << "Null QtProperty parent, not adding visual elements."
           << " This should never happen." << std::endl;
    return;
  }

  QtProperty *topItem = nullptr;
  QtVariantProperty *item = nullptr;

  // Name value
  item = this->dataPtr->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  _parent->addSubProperty(item);

  // Laser Retro value
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("laser_retro"));
  if (_msg.has_laser_retro())
    item->setValue(_msg.laser_retro());
  else
    item->setValue(0.0);
  _parent->addSubProperty(item);

  // cast shadows value
  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
                                           tr("cast_shadows"));
  if (_msg.has_cast_shadows())
    item->setValue(_msg.cast_shadows());
  else
    item->setValue(true);
  _parent->addSubProperty(item);

  // transparency
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("transparency"));
  if (_msg.has_transparency())
    item->setValue(_msg.transparency());
  else
    item->setValue(0.0);
  _parent->addSubProperty(item);


  // Pose value
  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("pose"));
  _parent->addSubProperty(topItem);
  this->FillPoseProperty(_msg.pose(), topItem);

  // Geometry shape value
  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("geometry"));
  _parent->addSubProperty(topItem);
  this->FillPropertyTree(_msg.geometry(), topItem);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Model &_msg,
                                       QtProperty * /*_parent*/)
{
  QtProperty *topItem = nullptr;
  QtVariantProperty *item = nullptr;

  item = this->dataPtr->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  this->dataPtr->propTreeBrowser->addProperty(item);
  // TODO: setting model name currently causes problems
  item->setEnabled(false);

  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
                                           tr("is_static"));
  if (_msg.has_is_static())
    item->setValue(_msg.is_static());
  else
    item->setValue(false);
  /// \todo Dynamically setting a model static doesn't currently work.
  item->setEnabled(false);
  this->dataPtr->propTreeBrowser->addProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
      tr("self_collide"));
  if (_msg.has_self_collide())
    item->setValue(_msg.self_collide());
  else
    item->setValue(false);
  item->setEnabled(false);
  this->dataPtr->propTreeBrowser->addProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
      tr("enable_wind"));
  if (_msg.has_enable_wind())
    item->setValue(_msg.enable_wind());
  else
    item->setValue(false);
  this->dataPtr->propTreeBrowser->addProperty(item);

  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  QtBrowserItem *bItem = this->dataPtr->propTreeBrowser->addProperty(topItem);
  this->dataPtr->propTreeBrowser->setExpanded(bItem, false);
  this->FillPoseProperty(_msg.pose(), topItem);

  for (int i = 0; i < _msg.link_size(); i++)
  {
    QtVariantProperty *prop;
    prop = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), tr("link"));
    prop->setToolTip(tr(_msg.link(i).name().c_str()));

    bItem = this->dataPtr->propTreeBrowser->addProperty(prop);
    this->dataPtr->propTreeBrowser->setExpanded(bItem, false);

    this->FillPropertyTree(_msg.link(i), prop);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Plugin &_msg,
                                       QtProperty *_parent)
{
  QtVariantProperty *item = nullptr;

  // name
  item = this->dataPtr->variantManager->addProperty(QVariant::String,
      tr("name"));
  item->setValue(_msg.name().c_str());
  item->setEnabled(false);
  this->AddProperty(item, _parent);

  // filename
  item = this->dataPtr->variantManager->addProperty(QVariant::String,
    tr("filename"));
  item->setValue(_msg.filename().c_str());
  item->setEnabled(false);
  this->AddProperty(item, _parent);

  // innerxml
  item = this->dataPtr->variantManager->addProperty(QVariant::String,
    tr("innerxml"));
  item->setValue(_msg.innerxml().c_str());
  item->setEnabled(false);
  this->AddProperty(item, _parent);
}

/////////////////////////////////////////////////
void ModelListWidget::FillVector3dProperty(const msgs::Vector3d &_msg,
                                           QtProperty *_parent)
{
  if (!_parent)
  {
    gzwarn << "Null QtProperty parent, not adding Vector3d elements."
           << " This should never happen." << std::endl;
    return;
  }

  QtVariantProperty *item;
  ignition::math::Vector3d value;
  value = msgs::ConvertIgn(_msg);
  value.Round(6);

  // Add X value
  item = static_cast<QtVariantProperty*>(this->ChildItem(_parent, "x"));
  if (!item)
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::Double, "x");
    _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  item->setValue(value.X());

  // Add Y value
  item = static_cast<QtVariantProperty*>(this->ChildItem(_parent, "y"));
  if (!item)
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::Double, "y");
    _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>(
        this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
            item, "decimals", 6);
  item->setValue(value.Y());

  // Add Z value
  item = static_cast<QtVariantProperty*>(this->ChildItem(_parent, "z"));
  if (!item)
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::Double, "z");
    _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>(
      this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
          item, "decimals", 6);
  item->setValue(value.Z());
}

/////////////////////////////////////////////////
void ModelListWidget::FillPoseProperty(const msgs::Pose &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
  {
    gzwarn << "Null QtProperty parent, not adding pose elements."
           << " This should never happen." << std::endl;
    return;
  }

  QtVariantProperty *item;
  ignition::math::Pose3d value;
  value = msgs::ConvertIgn(_msg);
  value.Round(6);

  ignition::math::Vector3d rpy = value.Rot().Euler();
  rpy.Round(6);

  this->FillVector3dProperty(_msg.position(), _parent);

  // Add Roll value
  item = static_cast<QtVariantProperty*>(this->ChildItem(_parent, "roll"));
  if (!item)
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::Double, "roll");
    _parent->addSubProperty(item);
    static_cast<QtVariantPropertyManager *>(
        this->dataPtr->variantFactory->propertyManager(
        item))->setAttribute(item, "decimals", 6);
    static_cast<QtVariantPropertyManager *>(
        this->dataPtr->variantFactory->propertyManager(
        item))->setAttribute(item, "singleStep", 0.05);
  }
  item->setValue(rpy.X());

  // Add Pitch value
  item = static_cast<QtVariantProperty*>(this->ChildItem(_parent, "pitch"));
  if (!item)
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
        "pitch");
    _parent->addSubProperty(item);
    static_cast<QtVariantPropertyManager *>(
        this->dataPtr->variantFactory->propertyManager(
        item))->setAttribute(item, "decimals", 6);
    static_cast<QtVariantPropertyManager *>(
        this->dataPtr->variantFactory->propertyManager(
        item))->setAttribute(item, "singleStep", 0.05);
  }
  item->setValue(rpy.Y());

  // Add Yaw value
  item = static_cast<QtVariantProperty*>(this->ChildItem(_parent, "yaw"));
  if (!item)
  {
    item = this->dataPtr->variantManager->addProperty(QVariant::Double, "yaw");
    _parent->addSubProperty(item);
    static_cast<QtVariantPropertyManager *>(
        this->dataPtr->variantFactory->propertyManager(
        item))->setAttribute(item, "decimals", 6);
    static_cast<QtVariantPropertyManager *>(
        this->dataPtr->variantFactory->propertyManager(
        item))->setAttribute(item, "singleStep", 0.05);
  }
  item->setValue(rpy.Z());
}

/////////////////////////////////////////////////
void ModelListWidget::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    this->dataPtr->removeEntityList.push_back(_msg->data());
  }
}

/////////////////////////////////////////////////
void ModelListWidget::ProcessRemoveEntity()
{
  for (auto iter = this->dataPtr->removeEntityList.begin();
       iter != this->dataPtr->removeEntityList.end(); ++iter)
  {
    this->RemoveEntity(*iter);
  }
  this->dataPtr->removeEntityList.clear();
}

/////////////////////////////////////////////////
void ModelListWidget::OnRemoveScene(const std::string &/*_name*/)
{
  this->ResetTree();
  this->dataPtr->propTreeBrowser->clear();
  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
  this->dataPtr->node.reset();

  this->dataPtr->requestPub.reset();
  this->dataPtr->modelPub.reset();
  this->dataPtr->scenePub.reset();
  this->dataPtr->physicsPub.reset();
  this->dataPtr->atmospherePub.reset();
  this->dataPtr->windPub.reset();
  this->dataPtr->lightPub.reset();
  this->dataPtr->responseSub.reset();
  this->dataPtr->requestSub.reset();
}

/////////////////////////////////////////////////
void ModelListWidget::OnCreateScene(const std::string &_name)
{
  this->ResetTree();

  this->dataPtr->propTreeBrowser->clear();
  this->InitTransport(_name);

  // this->requestMsg = msgs::CreateRequest("scene_info");
  // this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelListWidget::InitTransport(const std::string &_name)
{
  if (this->dataPtr->node)
  {
    this->dataPtr->node->Fini();
    this->dataPtr->node.reset();
    this->dataPtr->requestPub.reset();
    this->dataPtr->modelPub.reset();
    this->dataPtr->scenePub.reset();
    this->dataPtr->physicsPub.reset();
    this->dataPtr->atmospherePub.reset();
    this->dataPtr->windPub.reset();
    this->dataPtr->lightPub.reset();
    this->dataPtr->responseSub.reset();
    this->dataPtr->requestSub.reset();
  }

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(_name);

  this->dataPtr->modelPub = this->dataPtr->node->Advertise<msgs::Model>(
      "~/model/modify");
  this->dataPtr->scenePub = this->dataPtr->node->Advertise<msgs::Scene>(
      "~/scene");
  this->dataPtr->physicsPub = this->dataPtr->node->Advertise<msgs::Physics>(
      "~/physics");

  this->dataPtr->atmospherePub =
    this->dataPtr->node->Advertise<msgs::Atmosphere>("~/atmosphere");

  this->dataPtr->windPub = this->dataPtr->node->Advertise<msgs::Wind>(
      "~/wind");

  this->dataPtr->lightPub = this->dataPtr->node->Advertise<msgs::Light>(
      "~/light/modify");

  this->dataPtr->requestPub = this->dataPtr->node->Advertise<msgs::Request>(
      "~/request");
  this->dataPtr->responseSub = this->dataPtr->node->Subscribe("~/response",
                                            &ModelListWidget::OnResponse, this);

  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
      &ModelListWidget::OnRequest, this, false);
}

/////////////////////////////////////////////////
void ModelListWidget::ResetTree()
{
  this->dataPtr->modelTreeWidget->clear();

  // Create the top level of items in the tree widget
  {
    // GUI item
    this->dataPtr->guiItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("GUI"))));
    this->dataPtr->guiItem->setData(0, Qt::UserRole, QVariant(tr("GUI")));
    this->dataPtr->modelTreeWidget->addTopLevelItem(this->dataPtr->guiItem);

    // Scene item
    this->ResetScene();

    // Spherical coordinates item
    this->dataPtr->sphericalCoordItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Spherical Coordinates"))));
    this->dataPtr->sphericalCoordItem->setData(0,
        Qt::UserRole, QVariant(tr("Spherical Coordinates")));
    this->dataPtr->modelTreeWidget->addTopLevelItem(
        this->dataPtr->sphericalCoordItem);

    // Physics item
    this->dataPtr->physicsItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Physics"))));
    this->dataPtr->physicsItem->setData(0, Qt::UserRole,
        QVariant(tr("Physics")));
    this->dataPtr->modelTreeWidget->addTopLevelItem(this->dataPtr->physicsItem);

    // Atmosphere item
    this->dataPtr->atmosphereItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem *>(0),
        QStringList(QString("%1").arg(tr("Atmosphere"))));
    this->dataPtr->atmosphereItem->setData(0, Qt::UserRole,
        QVariant(tr("Atmosphere")));
    this->dataPtr->modelTreeWidget->addTopLevelItem(
        this->dataPtr->atmosphereItem);

    // Wind item
    this->dataPtr->windItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Wind"))));
    this->dataPtr->windItem->setData(0, Qt::UserRole, QVariant(tr("Wind")));
    this->dataPtr->modelTreeWidget->addTopLevelItem(this->dataPtr->windItem);

    // Models item
    this->dataPtr->modelsItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Models"))));
    this->dataPtr->modelsItem->setData(0, Qt::UserRole, QVariant(tr("Models")));
    this->dataPtr->modelTreeWidget->addTopLevelItem(this->dataPtr->modelsItem);

    // Lights item
    this->dataPtr->lightsItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Lights"))));
    this->dataPtr->lightsItem->setData(0, Qt::UserRole, QVariant(tr("Lights")));
    this->dataPtr->modelTreeWidget->addTopLevelItem(this->dataPtr->lightsItem);
  }

  this->dataPtr->fillingPropertyTree = false;
  this->dataPtr->selectedProperty = nullptr;
}

/////////////////////////////////////////////////
void ModelListWidget::ResetScene()
{
  this->dataPtr->sceneItem = new QTreeWidgetItem(
      static_cast<QTreeWidgetItem*>(0),
      QStringList(QString("%1").arg(tr("Scene"))));
  this->dataPtr->sceneItem->setData(0, Qt::UserRole, QVariant(tr("Scene")));
  this->dataPtr->modelTreeWidget->addTopLevelItem(this->dataPtr->sceneItem);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Scene &_msg,
                                       QtProperty * /*_parent*/)
{
  // QtProperty *topItem = nullptr;
  QtVariantProperty *item = nullptr;

  // Create and set the ambient color property
  item = this->dataPtr->variantManager->addProperty(QVariant::Color,
      tr("ambient"));
  if (_msg.has_ambient())
  {
    QColor clr(_msg.ambient().r()*255, _msg.ambient().g()*255,
               _msg.ambient().b()*255, _msg.ambient().a()*255);
    item->setValue(clr);
  }
  this->dataPtr->propTreeBrowser->addProperty(item);

  // Create and set the background color property
  item = this->dataPtr->variantManager->addProperty(QVariant::Color,
      tr("background"));
  if (_msg.has_background())
  {
    QColor clr(_msg.background().r()*255, _msg.background().g()*255,
               _msg.background().b()*255, _msg.background().a()*255);
    item->setValue(clr);
  }
  this->dataPtr->propTreeBrowser->addProperty(item);

  // Create and set the shadows property
  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
      tr("shadows"));
  if (_msg.has_shadows())
    item->setValue(_msg.shadows());
  this->dataPtr->propTreeBrowser->addProperty(item);

  /// \TODO: Put fog back in
  /*topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("fog"));
  QtBrowserItem *bItem = this->dataPtr->propTreeBrowser->addProperty(topItem);
  this->dataPtr->propTreeBrowser->setExpanded(bItem, false);

  item = this->dataPtr->variantManager->addProperty(QVariant::Color, tr("color"));
  topItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double, tr("start"));
  topItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double, tr("end"));
  topItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double, tr("density"));
  topItem->addSubProperty(item);
  */

  /// \TODO: Put sky modification back in GUI
  /*
  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("sky"));
  bItem = this->dataPtr->propTreeBrowser->addProperty(topItem);
  this->dataPtr->propTreeBrowser->setExpanded(bItem, false);
  */
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Physics &_msg,
                                       QtProperty * /*_parent*/)
{
  QtVariantProperty *item = nullptr;

  if (_msg.has_type())
  {
    this->dataPtr->physicsType = _msg.type();

    item = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::enumTypeId(), tr("physics engine"));
    QStringList types;

    const google::protobuf::EnumDescriptor *engineTypeEnum =
      _msg.GetDescriptor()->FindEnumTypeByName("Type");

    if (!engineTypeEnum)
    {
      gzerr << "Unable to get Type enum descriptor from "
        << "Physics message. msgs::Physics "
        << "has probably changed\n";
      types << "invalid";
    }
    else
    {
      types << engineTypeEnum->value(_msg.type()-1)->name().c_str();
    }

    item->setAttribute("enumNames", types);
    item->setValue(0);
    this->dataPtr->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
    tr("enable physics"));
  if (_msg.has_enable_physics())
    item->setValue(_msg.enable_physics());
  this->dataPtr->propTreeBrowser->addProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("real time update rate"));
  static_cast<QtVariantPropertyManager*>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_real_time_update_rate())
    item->setValue(_msg.real_time_update_rate());
  this->dataPtr->propTreeBrowser->addProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("max step size"));
  static_cast<QtVariantPropertyManager*>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_max_step_size())
    item->setValue(_msg.max_step_size());
  this->dataPtr->propTreeBrowser->addProperty(item);

  QtProperty *gravityItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("gravity"));
  this->dataPtr->propTreeBrowser->addProperty(gravityItem);
  if (_msg.has_gravity())
    this->FillVector3dProperty(_msg.gravity(), gravityItem);
  else
  {
    msgs::Vector3d xyz;
    xyz.set_x(0);
    xyz.set_y(0);
    xyz.set_z(-9.8);
    this->FillVector3dProperty(xyz, gravityItem);
  }

  QtProperty *magneticFieldItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("magnetic field"));
  this->dataPtr->propTreeBrowser->addProperty(magneticFieldItem);
  if (_msg.has_magnetic_field())
    this->FillVector3dProperty(_msg.magnetic_field(), magneticFieldItem);
  else
  {
    msgs::Vector3d xyz;
    xyz.set_x(0.0);
    xyz.set_y(0.0);
    xyz.set_z(0.0);
    this->FillVector3dProperty(xyz, magneticFieldItem);
  }

  QtProperty *solverItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("solver"));
  this->dataPtr->propTreeBrowser->addProperty(solverItem);

  item = this->dataPtr->variantManager->addProperty(QVariant::Int,
      tr("iterations"));
  if (_msg.has_iters())
    item->setValue(_msg.iters());
  solverItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("SOR"));
  static_cast<QtVariantPropertyManager*>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_sor())
    item->setValue(_msg.sor());
  solverItem->addSubProperty(item);


  QtProperty *constraintsItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("constraints"));
  this->dataPtr->propTreeBrowser->addProperty(constraintsItem);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("CFM"));
  static_cast<QtVariantPropertyManager*>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_cfm())
    item->setValue(_msg.cfm());
  constraintsItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("ERP"));
  static_cast<QtVariantPropertyManager*>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_erp())
    item->setValue(_msg.erp());
  constraintsItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("max velocity"));
  static_cast<QtVariantPropertyManager*>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_contact_max_correcting_vel())
    item->setValue(_msg.contact_max_correcting_vel());
  constraintsItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                           tr("surface layer"));
  static_cast<QtVariantPropertyManager*>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_contact_surface_layer())
    item->setValue(_msg.contact_surface_layer());
  constraintsItem->addSubProperty(item);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Atmosphere &_msg,
                                       QtProperty */*_parent*/)
{
  QtVariantProperty *item = nullptr;

  if (_msg.has_type())
    this->dataPtr->atmosphereType = _msg.type();

  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
    tr("enable atmosphere"));
  if (_msg.has_enable_atmosphere())
    item->setValue(_msg.enable_atmosphere());
  this->dataPtr->propTreeBrowser->addProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("temperature"));
  static_cast<QtVariantPropertyManager *>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_temperature())
    item->setValue(_msg.temperature());
  this->dataPtr->propTreeBrowser->addProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("pressure"));
  static_cast<QtVariantPropertyManager *>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_pressure())
    item->setValue(_msg.pressure());
  this->dataPtr->propTreeBrowser->addProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("mass_density"));
  static_cast<QtVariantPropertyManager *>
    (this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_mass_density())
    item->setValue(_msg.mass_density());
  item->setEnabled(false);
  this->dataPtr->propTreeBrowser->addProperty(item);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Wind &_msg,
                                       QtProperty * /*_parent*/)
{
  QtVariantProperty *item = nullptr;

  item = this->dataPtr->variantManager->addProperty(QVariant::Bool,
    tr("enable wind"));
  if (_msg.has_enable_wind())
    item->setValue(_msg.enable_wind());
  this->dataPtr->propTreeBrowser->addProperty(item);

  QtProperty *linearVelocityItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("linear_velocity"));
  this->dataPtr->propTreeBrowser->addProperty(linearVelocityItem);
  if (_msg.has_linear_velocity())
    this->FillVector3dProperty(_msg.linear_velocity(), linearVelocityItem);
  else
  {
    msgs::Vector3d xyz;
    xyz.set_x(0.0);
    xyz.set_y(0.0);
    xyz.set_z(0.0);
    this->FillVector3dProperty(xyz, linearVelocityItem);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Light &_msg,
                                       QtProperty * /*_parent*/)
{
  QtVariantProperty *item = nullptr;
  QtProperty *topItem = nullptr;

  this->dataPtr->lightType = _msg.type();

  item = this->dataPtr->variantManager->addProperty(QVariant::String,
      tr("name"));
  if (_msg.has_name())
    item->setValue(_msg.name().c_str());
  this->dataPtr->propTreeBrowser->addProperty(item);

  topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  this->dataPtr->propTreeBrowser->addProperty(topItem);
  if (_msg.has_pose())
    this->FillPoseProperty(_msg.pose(), topItem);
  else
    this->FillPoseProperty(msgs::Convert(ignition::math::Pose3d()), topItem);

  // Create and set the diffuse color property
  item = this->dataPtr->variantManager->addProperty(QVariant::Color,
      tr("diffuse"));
  if (_msg.has_diffuse())
  {
    QColor clr(_msg.diffuse().r()*255, _msg.diffuse().g()*255,
               _msg.diffuse().b()*255, _msg.diffuse().a()*255);
    item->setValue(clr);
  }
  this->dataPtr->propTreeBrowser->addProperty(item);

  // Create and set the specular color property
  item = this->dataPtr->variantManager->addProperty(QVariant::Color,
      tr("specular"));
  if (_msg.has_specular())
  {
    QColor clr(_msg.specular().r()*255, _msg.specular().g()*255,
               _msg.specular().b()*255, _msg.specular().a()*255);
    item->setValue(clr);
  }
  this->dataPtr->propTreeBrowser->addProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("range"));
  if (_msg.has_range())
    item->setValue(_msg.range());
  this->dataPtr->propTreeBrowser->addProperty(item);

  QtProperty *attenuationItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("attenuation"));
  this->dataPtr->propTreeBrowser->addProperty(attenuationItem);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("constant"));
  if (_msg.has_attenuation_constant())
    item->setValue(_msg.attenuation_constant());
  attenuationItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("linear"));
  if (_msg.has_attenuation_linear())
    item->setValue(_msg.attenuation_linear());
  attenuationItem->addSubProperty(item);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("quadratic"));
  if (_msg.has_attenuation_quadratic())
    item->setValue(_msg.attenuation_quadratic());
  attenuationItem->addSubProperty(item);

  if (_msg.has_spot_inner_angle() || _msg.has_spot_outer_angle() ||
      _msg.has_spot_falloff())
  {
    QtProperty *spotItem = this->dataPtr->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), tr("spot light"));
    this->dataPtr->propTreeBrowser->addProperty(spotItem);

    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                             tr("inner angle"));
    if (_msg.has_spot_inner_angle())
      item->setValue(_msg.spot_inner_angle());
    spotItem->addSubProperty(item);

    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                             tr("outer angle"));
    if (_msg.has_spot_outer_angle())
      item->setValue(_msg.spot_outer_angle());
    spotItem->addSubProperty(item);

    item = this->dataPtr->variantManager->addProperty(QVariant::Double,
                                             tr("falloff"));
    if (_msg.has_spot_falloff())
      item->setValue(_msg.spot_falloff());
    spotItem->addSubProperty(item);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::ProcessLightMsgs()
{
  std::lock_guard<std::mutex> lock(*this->dataPtr->receiveMutex);

  for (auto iter = this->dataPtr->lightMsgs.begin();
       iter != this->dataPtr->lightMsgs.end(); ++iter)
  {
    std::string name = (*iter).name();

    QTreeWidgetItem *listItem = this->ListItem((*iter).name(),
                                                  this->dataPtr->lightsItem);

    if (!listItem)
    {
      // Create a top-level tree item for the path
      QTreeWidgetItem *item = new QTreeWidgetItem(this->dataPtr->lightsItem,
          QStringList(QString("%1").arg(QString::fromStdString(name))));

      item->setData(0, Qt::UserRole, QVariant((*iter).name().c_str()));
      this->dataPtr->modelTreeWidget->addTopLevelItem(item);
    }
    else
    {
      listItem->setData(0, Qt::UserRole, QVariant((*iter).name().c_str()));
    }
  }
  this->dataPtr->lightMsgs.clear();
}

/////////////////////////////////////////////////
void ModelListWidget::AddProperty(QtProperty *_item, QtProperty *_parent)
{
  if (!_item)
  {
    gzwarn << "Null QtProperty item, not adding."
           << " This should never happen." << std::endl;
    return;
  }

  if (_parent)
    _parent->addSubProperty(_item);
  else
    this->dataPtr->propTreeBrowser->addProperty(_item);
}

/////////////////////////////////////////////////
void ModelListWidget::FillUserCamera()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (!cam)
    return;

  QtVariantProperty *item = nullptr;

  // Create a camera item
  QtProperty *topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("camera"));
  auto cameraBrowser = this->dataPtr->propTreeBrowser->addProperty(topItem);

  // Create and set the gui camera name
  std::string cameraName = cam->Name();
  item = this->dataPtr->variantManager->addProperty(QVariant::String,
      tr("name"));
  item->setValue(cameraName.c_str());
  topItem->addSubProperty(item);
  item->setEnabled(false);

  // Create and set the gui camera clip distance items
  auto clipItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("clip"));
  topItem->addSubProperty(clipItem);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("near"));
  item->setValue(cam->NearClip());
  clipItem->addSubProperty(item);
  item->setEnabled(true);

  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("far"));
  item->setValue(cam->FarClip());
  clipItem->addSubProperty(item);
  item->setEnabled(true);

  // Create and set the gui camera pose
  item = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  {
    topItem->addSubProperty(item);
    ignition::math::Pose3d cameraPose = cam->WorldPose();

    this->FillPoseProperty(msgs::Convert(cameraPose), item);
    // set expanded to true by default for easier viewing
    this->dataPtr->propTreeBrowser->setExpanded(cameraBrowser, true);
    for (auto browser : cameraBrowser->children())
    {
      this->dataPtr->propTreeBrowser->setExpanded(browser, true);
    }
  }

  // Create and set the gui camera position relative to a tracked model
  item = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("track_visual"));
  {
    topItem->addSubProperty(item);

    rendering::VisualPtr trackedVisual = cam->TrackedVisual();
    QtVariantProperty *item2 = this->dataPtr->variantManager->addProperty(
        QVariant::String, tr("name"));
    if (trackedVisual)
        item2->setValue(trackedVisual->GetName().c_str());
    else
        item2->setValue("");
    item2->setEnabled(false);
    item->addSubProperty(item2);

    bool isStatic = cam->TrackIsStatic();
    item2 = this->dataPtr->variantManager->addProperty(
        QVariant::Bool, tr("static"));
    item2->setValue(isStatic);
    item->addSubProperty(item2);

    bool useModelFrame = cam->TrackUseModelFrame();
    item2 = this->dataPtr->variantManager->addProperty(
        QVariant::Bool, tr("use_model_frame"));
    item2->setValue(useModelFrame);
    item->addSubProperty(item2);

    bool inheritYaw = cam->TrackInheritYaw();
    item2 = this->dataPtr->variantManager->addProperty(
        QVariant::Bool, tr("inherit_yaw"));
    item2->setValue(inheritYaw);
    item->addSubProperty(item2);

    ignition::math::Vector3d trackPos = cam->TrackPosition();
    this->FillVector3dProperty(msgs::Convert(trackPos), item);

    double minDist = cam->TrackMinDistance();
    item2 = this->dataPtr->variantManager->addProperty(
        QVariant::Double, tr("min_distance"));
    static_cast<QtVariantPropertyManager*>
      (this->dataPtr->variantFactory->propertyManager(item2))->setAttribute(
          item2, "decimals", 6);
    item2->setValue(minDist);
    item->addSubProperty(item2);

    double maxDist = cam->TrackMaxDistance();
    item2 = this->dataPtr->variantManager->addProperty(
        QVariant::Double, tr("max_distance"));
    static_cast<QtVariantPropertyManager*>
      (this->dataPtr->variantFactory->propertyManager(item2))->setAttribute(
          item2, "decimals", 6);
    item2->setValue(maxDist);
    item->addSubProperty(item2);
  }

  // set expanded to true by default for easier viewing
  this->dataPtr->propTreeBrowser->setExpanded(cameraBrowser, true);
  for (auto browser : cameraBrowser->children())
  {
    this->dataPtr->propTreeBrowser->setExpanded(browser, true);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::FillGrid()
{
  auto scene = rendering::get_scene();
  if (!scene)
    return;

  // Get the main grid
  auto grid = scene->GetGrid(0);
  if (!grid)
    return;

  QtVariantProperty *item = nullptr;

  // Top level grid item
  auto topItem = this->dataPtr->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("grid"));
  this->dataPtr->propTreeBrowser->addProperty(topItem);

  // Cell count
  auto cellCount = grid->CellCount();
  item = this->dataPtr->variantManager->addProperty(QVariant::Int,
      tr("cell count"));
  static_cast<QtVariantPropertyManager *>(
      this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
      item, "minimum", 0);
  item->setValue(cellCount);
  topItem->addSubProperty(item);

  // Cell size
  auto cellLength = grid->CellLength();
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("cell size"));
  static_cast<QtVariantPropertyManager *>(
      this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
      item, "minimum", 0);
  static_cast<QtVariantPropertyManager *>(
      this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
      item, "decimals", 4);
  item->setValue(cellLength);
  topItem->addSubProperty(item);

  // Normal cell count
  auto height = grid->Height();
  item = this->dataPtr->variantManager->addProperty(QVariant::Int,
      tr("normal cell count"));
  static_cast<QtVariantPropertyManager *>(
      this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
      item, "minimum", 0);
  item->setValue(height);
  topItem->addSubProperty(item);

  // Height offset
  auto heightOffset = grid->HeightOffset();
  item = this->dataPtr->variantManager->addProperty(QVariant::Double,
      tr("height offset"));
  static_cast<QtVariantPropertyManager *>(
      this->dataPtr->variantFactory->propertyManager(item))->setAttribute(
      item, "decimals", 4);
  item->setValue(heightOffset);
  topItem->addSubProperty(item);

  // Line color
  auto color = grid->Color();
  item = this->dataPtr->variantManager->addProperty(QVariant::Color,
      tr("line color"));
  item->setValue(gui::Conversions::Convert(color));
  topItem->addSubProperty(item);
}
