/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/mutex.hpp>

#include <sdf/sdf.hh>
#include "gazebo/common/Image.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/gui/GuiIface.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PhysicsTypes.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Helpers.hh"

#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/ModelRightMenu.hh"
#include "gazebo/gui/qtpropertybrowser/qttreepropertybrowser.h"
#include "gazebo/gui/qtpropertybrowser/qtvariantproperty.h"
#include "gazebo/gui/ModelListWidget.hh"

// avoid collision from Mac OS X's ConditionalMacros.h
#ifdef __MACH__
#undef TYPE_BOOL
#endif

using namespace gazebo;
using namespace gui;

extern ModelRightMenu *g_modelRightMenu;

/////////////////////////////////////////////////
ModelListWidget::ModelListWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("modelList");

  this->requestMsg = NULL;
  this->propMutex = new boost::mutex();
  this->receiveMutex = new boost::mutex();

  QVBoxLayout *mainLayout = new QVBoxLayout;
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

  connect(this->modelTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)),
          this, SLOT(OnModelSelection(QTreeWidgetItem *, int)));
  connect(this->modelTreeWidget,
      SIGNAL(customContextMenuRequested(const QPoint &)),
      this, SLOT(OnCustomContextMenu(const QPoint &)));

  this->variantManager = new QtVariantPropertyManager();
  this->propTreeBrowser = new QtTreePropertyBrowser();
  this->propTreeBrowser->setObjectName("propTreeBrowser");
  this->propTreeBrowser->setStyleSheet(
      "QTreeView::branch:selected:active { background-color: transparent; }");
  this->variantFactory = new QtVariantEditorFactory();
  this->propTreeBrowser->setFactoryForManager(this->variantManager,
                                              this->variantFactory);
  connect(this->variantManager,
          SIGNAL(propertyChanged(QtProperty*)),
          this, SLOT(OnPropertyChanged(QtProperty *)));
  connect(this->propTreeBrowser,
          SIGNAL(currentItemChanged(QtBrowserItem*)),
          this, SLOT(OnCurrentPropertyChanged(QtBrowserItem *)));

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;

  QSplitter *splitter = new QSplitter(Qt::Vertical, this);
  splitter->addWidget(this->modelTreeWidget);
  splitter->addWidget(this->propTreeBrowser);
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

  this->connections.push_back(
      gui::Events::ConnectModelUpdate(
        boost::bind(&ModelListWidget::OnModelUpdate, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectLightUpdate(
        boost::bind(&ModelListWidget::OnLightUpdate, this, _1)));

  this->connections.push_back(
      rendering::Events::ConnectCreateScene(
        boost::bind(&ModelListWidget::OnCreateScene, this, _1)));

  this->connections.push_back(
      rendering::Events::ConnectRemoveScene(
        boost::bind(&ModelListWidget::OnRemoveScene, this, _1)));

  this->connections.push_back(
      event::Events::ConnectSetSelectedEntity(
        boost::bind(&ModelListWidget::OnSetSelectedEntity, this, _1, _2)));

  QTimer::singleShot(500, this, SLOT(Update()));
}

/////////////////////////////////////////////////
ModelListWidget::~ModelListWidget()
{
  this->connections.clear();
  delete this->propMutex;
  delete this->receiveMutex;
}

/////////////////////////////////////////////////
void ModelListWidget::OnModelSelection(QTreeWidgetItem *_item, int /*_column*/)
{
  if (_item)
  {
    std::string name = _item->data(0, Qt::UserRole).toString().toStdString();
    this->propTreeBrowser->clear();
    if (name == "Scene")
    {
      this->requestMsg = msgs::CreateRequest("scene_info",
                         this->selectedEntityName);
      this->requestPub->Publish(*this->requestMsg);
    }
    else if (name == "Models")
    {
      this->modelsItem->setExpanded(!this->modelsItem->isExpanded());
    }
    else if (name == "Lights")
    {
      this->lightsItem->setExpanded(!this->lightsItem->isExpanded());
    }
    else if (name == "Physics")
    {
      this->requestMsg = msgs::CreateRequest("physics_info",
                                             this->selectedEntityName);
      this->requestPub->Publish(*this->requestMsg);
    }
    else if (name == "Spherical Coordinates")
    {
      this->requestMsg = msgs::CreateRequest("spherical_coordinates_info",
                                             this->selectedEntityName);
      this->requestPub->Publish(*this->requestMsg);
    }
    else if (name == "GUI")
    {
      QtVariantProperty *item = NULL;

      rendering::UserCameraPtr cam = gui::get_active_camera();
      if (!cam)
        return;

      // Create a camera item
      QtProperty *topItem = this->variantManager->addProperty(
          QtVariantPropertyManager::groupTypeId(), tr("camera"));
      auto cameraBrowser = this->propTreeBrowser->addProperty(topItem);

      // Create and set the gui camera name
      std::string cameraName = cam->GetName();
      item = this->variantManager->addProperty(QVariant::String, tr("name"));
      item->setValue(cameraName.c_str());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // Create and set the gui camera pose
      item = this->variantManager->addProperty(
          QtVariantPropertyManager::groupTypeId(), tr("pose"));
      {
        topItem->addSubProperty(item);
        math::Pose cameraPose = cam->GetWorldPose();

        this->FillPoseProperty(msgs::Convert(cameraPose), item);
        // set expanded to true by default for easier viewing
        this->propTreeBrowser->setExpanded(cameraBrowser, true);
        for (auto browser : cameraBrowser->children())
        {
          this->propTreeBrowser->setExpanded(browser, true);
        }
      }
    }
    else
    {
      this->propTreeBrowser->clear();
      event::Events::setSelectedEntity(name, "normal");
    }
  }
  else
    this->selectedEntityName.clear();
}

/////////////////////////////////////////////////
void ModelListWidget::OnSetSelectedEntity(const std::string &_name,
                                          const std::string &/*_mode*/)
{
  this->selectedEntityName = _name;

  this->propTreeBrowser->clear();
  if (!this->selectedEntityName.empty())
  {
    QTreeWidgetItem *mItem = this->GetListItem(this->selectedEntityName,
                                               this->modelsItem);
    QTreeWidgetItem *lItem = this->GetListItem(this->selectedEntityName,
                                               this->lightsItem);

    if (mItem)
    {
      this->requestMsg = msgs::CreateRequest("entity_info",
          this->selectedEntityName);
      this->requestPub->Publish(*this->requestMsg);
      this->modelTreeWidget->setCurrentItem(mItem);
      mItem->setExpanded(!mItem->isExpanded());
    }
    else if (lItem)
    {
      rendering::LightPtr light =
        gui::get_active_camera()->GetScene()->GetLight(
            this->selectedEntityName);

      light->FillMsg(this->lightMsg);
      this->propTreeBrowser->clear();
      this->fillTypes.push_back("Light");

      this->modelTreeWidget->setCurrentItem(lItem);
    }
  }
  else if (this->modelTreeWidget->currentItem())
  {
    this->modelTreeWidget->currentItem()->setSelected(false);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::Update()
{
  if (!this->fillTypes.empty())
  {
    boost::mutex::scoped_lock lock(*this->propMutex);
    this->fillingPropertyTree = true;
    this->propTreeBrowser->clear();

    if (this->fillTypes[0] == "Model")
      this->FillPropertyTree(this->modelMsg, NULL);
    else if (this->fillTypes[0] == "Link")
      this->FillPropertyTree(this->linkMsg, NULL);
    else if (this->fillTypes[0] == "Joint")
      this->FillPropertyTree(this->jointMsg, NULL);
    else if (this->fillTypes[0] == "Scene")
      this->FillPropertyTree(this->sceneMsg, NULL);
    else if (this->fillTypes[0] == "Physics")
      this->FillPropertyTree(this->physicsMsg, NULL);
    else if (this->fillTypes[0] == "Light")
      this->FillPropertyTree(this->lightMsg, NULL);
    else if (this->fillTypes[0] == "Spherical Coordinates")
      this->FillPropertyTree(this->sphericalCoordMsg, NULL);

    this->fillingPropertyTree = false;
    this->fillTypes.pop_front();
  }

  if (!this->modelTreeWidget->currentItem())
  {
    boost::mutex::scoped_lock lock(*this->propMutex);
    this->propTreeBrowser->clear();
  }

  this->ProcessRemoveEntity();
  this->ProcessModelMsgs();
  this->ProcessLightMsgs();
  QTimer::singleShot(1000, this, SLOT(Update()));
}

/////////////////////////////////////////////////
void ModelListWidget::OnModelUpdate(const msgs::Model &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Model msg;
  msg.CopyFrom(_msg);
  this->modelMsgs.push_back(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::OnLightUpdate(const msgs::Light &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  msgs::Light msg;
  msg.CopyFrom(_msg);
  this->lightMsgs.push_back(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::ProcessModelMsgs()
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  for (ModelMsgs_L::iterator iter = this->modelMsgs.begin();
       iter != this->modelMsgs.end(); ++iter)
  {
    std::string name = (*iter).name();

    QTreeWidgetItem *listItem = this->GetListItem((*iter).name(),
                                                  this->modelsItem);

    if (!listItem)
    {
      if (!(*iter).has_deleted() || !(*iter).deleted())
      {
        // Create an item for the model name
        QTreeWidgetItem *topItem = new QTreeWidgetItem(this->modelsItem,
            QStringList(QString("%1").arg(QString::fromStdString(name))));

        topItem->setData(0, Qt::UserRole, QVariant((*iter).name().c_str()));
        this->modelTreeWidget->addTopLevelItem(topItem);

        for (int i = 0; i < (*iter).link_size(); i++)
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
          this->modelTreeWidget->addTopLevelItem(linkItem);
        }

        for (int i = 0; i < (*iter).joint_size(); i++)
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
          this->modelTreeWidget->addTopLevelItem(jointItem);
        }
      }
    }
    else
    {
      if ((*iter).has_deleted() && (*iter).deleted())
      {
        int i = this->modelsItem->indexOfChild(listItem);
        this->modelsItem->takeChild(i);
      }
      else
      {
        listItem->setText(0, (*iter).name().c_str());
        listItem->setData(1, Qt::UserRole, QVariant((*iter).name().c_str()));
      }
    }
  }
  this->modelMsgs.clear();
}

/////////////////////////////////////////////////
void ModelListWidget::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  if (_msg->has_type() && _msg->type() == this->modelMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->modelMsg.ParseFromString(_msg->serialized_data());
    this->fillTypes.push_back("Model");
    this->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == this->linkMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->linkMsg.ParseFromString(_msg->serialized_data());
    this->fillTypes.push_back("Link");
    this->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == this->jointMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->jointMsg.ParseFromString(_msg->serialized_data());
    this->fillTypes.push_back("Joint");
    this->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == this->sceneMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->sceneMsg.ParseFromString(_msg->serialized_data());
    this->fillTypes.push_back("Scene");
    this->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == this->physicsMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->physicsMsg.ParseFromString(_msg->serialized_data());
    this->fillTypes.push_back("Physics");
    this->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == this->lightMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->lightMsg.ParseFromString(_msg->serialized_data());
    this->fillTypes.push_back("Light");
    this->propMutex->unlock();
  }
  else if (_msg->has_type() &&
           _msg->type() == this->sphericalCoordMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->sphericalCoordMsg.ParseFromString(_msg->serialized_data());
    this->fillTypes.push_back("Spherical Coordinates");
    this->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == "error")
  {
    if (_msg->response() == "nonexistent")
    {
      this->removeEntityList.push_back(this->selectedEntityName);
    }
  }

  delete this->requestMsg;
  this->requestMsg = NULL;
}

/////////////////////////////////////////////////
void ModelListWidget::RemoveEntity(const std::string &_name)
{
  QTreeWidgetItem *items[2];
  items[0] = this->modelsItem;
  items[1] = this->lightsItem;

  for (int i = 0; i < 2; ++i)
  {
    QTreeWidgetItem *listItem = this->GetListItem(_name, items[i]);
    if (listItem)
    {
      items[i]->takeChild(items[i]->indexOfChild(listItem));
      this->propTreeBrowser->clear();
      this->selectedEntityName.clear();
      this->sdfElement.reset();
      this->fillTypes.clear();
      return;
    }
  }
}

/////////////////////////////////////////////////
QTreeWidgetItem *ModelListWidget::GetListItem(const std::string &_name,
                                              QTreeWidgetItem *_parent)
{
  QTreeWidgetItem *listItem = NULL;

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
  QTreeWidgetItem *item = this->modelTreeWidget->itemAt(_pt);

  // Check to see if the selected item is a model
  int i = this->modelsItem->indexOfChild(item);
  if (i >= 0)
  {
    g_modelRightMenu->Run(item->text(0).toStdString(),
                          this->modelTreeWidget->mapToGlobal(_pt),
                          ModelRightMenu::EntityTypes::MODEL);
    return;
  }

  // Check to see if the selected item is a light
  i = this->lightsItem->indexOfChild(item);
  if (i >= 0)
  {
    g_modelRightMenu->Run(item->text(0).toStdString(),
                          this->modelTreeWidget->mapToGlobal(_pt),
                          ModelRightMenu::EntityTypes::LIGHT);
  }

  // Check to see if the selected item is a link
  if (item->data(3, Qt::UserRole).toString().toStdString() == "Link")
  {
    g_modelRightMenu->Run(item->data(0, Qt::UserRole).toString().toStdString(),
                          this->modelTreeWidget->mapToGlobal(_pt),
                          ModelRightMenu::EntityTypes::LINK);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::OnCurrentPropertyChanged(QtBrowserItem *_item)
{
  if (_item)
    this->selectedProperty = _item->property();
  else
    this->selectedProperty = NULL;
}

/////////////////////////////////////////////////
void ModelListWidget::OnPropertyChanged(QtProperty *_item)
{
  boost::mutex::scoped_try_lock lock(*this->propMutex);
  if (!lock)
    return;

  if (this->selectedProperty != _item || this->fillingPropertyTree)
    return;

  QTreeWidgetItem *currentItem = this->modelTreeWidget->currentItem();

  if (!currentItem)
    return;

  if (this->modelsItem->indexOfChild(currentItem) != -1 ||
      this->modelsItem->indexOfChild(currentItem->parent()) != -1)
    this->ModelPropertyChanged(_item);
  else if (this->lightsItem->indexOfChild(currentItem) != -1)
    this->LightPropertyChanged(_item);
  else if (currentItem == this->sceneItem)
    this->ScenePropertyChanged(_item);
  else if (currentItem == this->physicsItem)
    this->PhysicsPropertyChanged(_item);
  else if (currentItem == this->guiItem)
    this->GUIPropertyChanged(_item);
}

/////////////////////////////////////////////////
void ModelListWidget::LightPropertyChanged(QtProperty * /*_item*/)
{
  msgs::Light msg;

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == "name")
      msg.set_name(this->variantManager->value(
            (*iter)).toString().toStdString());
    else if ((*iter)->propertyName().toStdString() == "pose")
    {
      math::Pose pose;
      pose.Set(this->variantManager->value(
                 this->GetChildItem((*iter), "x")).toDouble(),
               this->variantManager->value(
                 this->GetChildItem((*iter), "y")).toDouble(),
               this->variantManager->value(
                 this->GetChildItem((*iter), "z")).toDouble(),
               this->variantManager->value(
                 this->GetChildItem((*iter), "roll")).toDouble(),
               this->variantManager->value(
                 this->GetChildItem((*iter), "pitch")).toDouble(),
               this->variantManager->value(
                 this->GetChildItem((*iter), "yaw")).toDouble());
      msgs::Set(msg.mutable_pose(), pose);
    }
    else if ((*iter)->propertyName().toStdString() == "range")
      msg.set_range(this->variantManager->value((*iter)).toDouble());
    else if ((*iter)->propertyName().toStdString() == "diffuse")
      this->FillColorMsg((*iter), msg.mutable_diffuse());
    else if ((*iter)->propertyName().toStdString() == "specular")
      this->FillColorMsg((*iter), msg.mutable_specular());
    else if ((*iter)->propertyName().toStdString() == "attenuation")
    {
      msg.set_attenuation_constant(this->variantManager->value(
            this->GetChildItem((*iter), "constant")).toDouble());
      msg.set_attenuation_linear(this->variantManager->value(
            this->GetChildItem((*iter), "linear")).toDouble());
      msg.set_attenuation_quadratic(this->variantManager->value(
            this->GetChildItem((*iter), "quadratic")).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "spot light")
    {
      msg.set_spot_inner_angle(this->variantManager->value(
            this->GetChildItem((*iter), "inner angle")).toDouble());
      msg.set_spot_outer_angle(this->variantManager->value(
            this->GetChildItem((*iter), "outer angle")).toDouble());
      msg.set_spot_falloff(this->variantManager->value(
            this->GetChildItem((*iter), "falloff")).toDouble());
    }
  }

  /// \TODO: Allow users to change light type
  msg.set_type(this->lightType);

  this->lightPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::GUIPropertyChanged(QtProperty *_item)
{
  // Only camera pose editable for now
  QtProperty *cameraProperty = this->GetChildItem("camera");
  if (!cameraProperty)
    return;

  QtProperty *cameraPoseProperty = this->GetChildItem(cameraProperty, "pose");
  if (!cameraPoseProperty)
    return;

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
        cam->SetWorldPose(msgs::Convert(poseMsg));
    }
  }
}

/////////////////////////////////////////////////
void ModelListWidget::PhysicsPropertyChanged(QtProperty * /*_item*/)
{
  msgs::Physics msg;

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == "gravity")
      this->FillVector3Msg((*iter), msg.mutable_gravity());
    else if ((*iter)->propertyName().toStdString() == "magnetic field")
      this->FillVector3Msg((*iter), msg.mutable_magnetic_field());
    else if ((*iter)->propertyName().toStdString() == "enable physics")
      msg.set_enable_physics(this->variantManager->value((*iter)).toBool());
    else if ((*iter)->propertyName().toStdString() == "solver")
    {
      msg.set_iters(this->variantManager->value(
            this->GetChildItem((*iter), "iterations")).toInt());
      msg.set_sor(this->variantManager->value(
            this->GetChildItem((*iter), "SOR")).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "constraints")
    {
      msg.set_cfm(this->variantManager->value(
            this->GetChildItem((*iter), "CFM")).toDouble());
      msg.set_erp(this->variantManager->value(
            this->GetChildItem((*iter), "ERP")).toDouble());
      msg.set_contact_max_correcting_vel(this->variantManager->value(
            this->GetChildItem((*iter), "max velocity")).toDouble());
      msg.set_contact_surface_layer(this->variantManager->value(
            this->GetChildItem((*iter), "surface layer")).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "real time update rate")
    {
      msg.set_real_time_update_rate(
          this->variantManager->value((*iter)).toDouble());
    }
    else if ((*iter)->propertyName().toStdString() == "max step size")
    {
      msg.set_max_step_size(this->variantManager->value((*iter)).toDouble());
    }
  }

  msg.set_type(this->physicsType);
  this->physicsPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::ScenePropertyChanged(QtProperty */*_item*/)
{
  msgs::Scene msg;

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
       iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == "ambient")
      this->FillColorMsg((*iter), msg.mutable_ambient());
    else if ((*iter)->propertyName().toStdString() == "background")
      this->FillColorMsg((*iter), msg.mutable_background());
    else if ((*iter)->propertyName().toStdString() == "shadows")
      msg.set_shadows(this->variantManager->value((*iter)).toBool());
  }

  msg.set_name(gui::get_world());
  this->scenePub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::ModelPropertyChanged(QtProperty *_item)
{
  msgs::Model msg;

  google::protobuf::Message *fillMsg = &msg;

  QTreeWidgetItem *currentItem = this->modelTreeWidget->currentItem();

  // check if it's a link
  if (currentItem->data(3, Qt::UserRole).toString().toStdString() == "Link")
  {
    // this->modelMsg may not have been set
    // so get the model name from the current item
    msg.set_name(currentItem->data(1, Qt::UserRole).toString().toStdString());
    msg.set_id(currentItem->data(2, Qt::UserRole).toInt());

    // set link id and strip link name.
    msgs::Link *linkMsg = msg.add_link();
    linkMsg->set_id(this->linkMsg.id());
    std::string linkName = this->linkMsg.name();
    size_t index = linkName.find_last_of("::");
    if (index != std::string::npos)
      linkName = linkName.substr(index+1);
    linkMsg->set_name(linkName);
    fillMsg = linkMsg;
  }
  else
  {
    msg.set_id(this->modelMsg.id());
    msg.set_name(this->modelMsg.name());
  }

  const google::protobuf::Descriptor *descriptor = fillMsg->GetDescriptor();
  const google::protobuf::Reflection *reflection = fillMsg->GetReflection();

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
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

  this->modelPub->Publish(msg);
}

/////////////////////////////////////////////////
void ModelListWidget::FillMsgField(QtProperty *_item,
    google::protobuf::Message *_message,
    const google::protobuf::Reflection *_reflection,
    const google::protobuf::FieldDescriptor *_field)
{
  if (_field->type() == google::protobuf::FieldDescriptor::TYPE_INT32)
    _reflection->SetInt32(_message, _field,
        this->variantManager->value(_item).toInt());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_DOUBLE)
    _reflection->SetDouble(_message, _field,
        this->variantManager->value(_item).toDouble());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_FLOAT)
    _reflection->SetFloat(_message, _field,
        this->variantManager->value(_item).toDouble());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_BOOL)
    _reflection->SetBool(_message, _field,
        this->variantManager->value(_item).toBool());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_STRING)
    _reflection->SetString(_message, _field,
        this->variantManager->value(_item).toString().toStdString());
  else if (_field->type() == google::protobuf::FieldDescriptor::TYPE_UINT32)
    _reflection->SetUInt32(_message, _field,
        this->variantManager->value(_item).toUInt());
  else
    gzerr << "Unable to fill message field[" << _field->type() << "]\n";
}

/////////////////////////////////////////////////
void ModelListWidget::FillColorMsg(QtProperty *_item, msgs::Color *_msg)
{
  _msg->set_r(this->variantManager->value(
      this->GetChildItem(_item, "Red")).toDouble() / 255.0);
  _msg->set_g(this->variantManager->value(
      this->GetChildItem(_item, "Green")).toDouble() / 255.0);
  _msg->set_b(this->variantManager->value(
      this->GetChildItem(_item, "Blue")).toDouble() / 255.0);
  _msg->set_a(this->variantManager->value(
      this->GetChildItem(_item, "Alpha")).toDouble() / 255.0);
}

/////////////////////////////////////////////////
void ModelListWidget::FillVector3Msg(QtProperty *_item, msgs::Vector3d *_msg)
{
  _msg->set_x(this->variantManager->value(
      this->GetChildItem(_item, "x")).toDouble());
  _msg->set_y(this->variantManager->value(
      this->GetChildItem(_item, "y")).toDouble());
  _msg->set_z(this->variantManager->value(
      this->GetChildItem(_item, "z")).toDouble());
}

/////////////////////////////////////////////////
void ModelListWidget::FillGeometryMsg(QtProperty *_item,
    google::protobuf::Message *_message,
    const google::protobuf::Descriptor *_descriptor,
    QtProperty * /*_changedItem*/)
{
  QtProperty *typeProperty = this->GetChildItem(_item, "type");

  std::string type = typeProperty->valueText().toStdString();
  const google::protobuf::Reflection *reflection = _message->GetReflection();
  reflection->SetEnum(_message, _descriptor->FindFieldByName("type"),
      _descriptor->FindEnumValueByName(type));

  boost::to_lower(type);
  const google::protobuf::FieldDescriptor *field =
    _descriptor->FindFieldByName(type);
  google::protobuf::Message *message =
    _message->GetReflection()->MutableMessage(_message, field);

  if (type == "box")
  {
    QtProperty *sizeProperty = this->GetChildItem(_item, "size");
    msgs::BoxGeom *boxMsg = (msgs::BoxGeom*)(message);
    double xValue = this->variantManager->value(
        this->GetChildItem(sizeProperty, "x")).toDouble();
    double yValue = this->variantManager->value(
        this->GetChildItem(sizeProperty, "y")).toDouble();
    double zValue = this->variantManager->value(
        this->GetChildItem(sizeProperty, "z")).toDouble();

    boxMsg->mutable_size()->set_x(xValue);
    boxMsg->mutable_size()->set_y(yValue);
    boxMsg->mutable_size()->set_z(zValue);
  }
  else if (type == "sphere")
  {
    QtProperty *radiusProperty = this->GetChildItem(_item, "radius");
    msgs::SphereGeom *sphereMsg = (msgs::SphereGeom*)(message);

    sphereMsg->set_radius(
        this->variantManager->value(radiusProperty).toDouble());
  }
  else if (type == "cylinder")
  {
    QtProperty *radiusProperty = this->GetChildItem(_item, "radius");
    QtProperty *lengthProperty = this->GetChildItem(_item, "length");

    msgs::CylinderGeom *cylinderMsg = (msgs::CylinderGeom*)(message);
    cylinderMsg->set_radius(
        this->variantManager->value(radiusProperty).toDouble());
    cylinderMsg->set_length(
        this->variantManager->value(lengthProperty).toDouble());
  }
  else if (type == "plane")
  {
    QtProperty *normalProperty = this->GetChildItem(_item, "normal");
    msgs::PlaneGeom *planeMessage = (msgs::PlaneGeom*)(message);

    double xValue = this->variantManager->value(
        this->GetChildItem(normalProperty, "x")).toDouble();
    double yValue = this->variantManager->value(
        this->GetChildItem(normalProperty, "y")).toDouble();
    double zValue = this->variantManager->value(
        this->GetChildItem(normalProperty, "z")).toDouble();

    planeMessage->mutable_normal()->set_x(xValue);
    planeMessage->mutable_normal()->set_y(yValue);
    planeMessage->mutable_normal()->set_z(zValue);
  }
  else if (type == "image")
  {
    QtProperty *fileProp = this->GetChildItem(_item, "filename");
    QtProperty *scaleProp = this->GetChildItem(_item, "scale");
    QtProperty *heightProp = this->GetChildItem(_item, "height");
    QtProperty *thresholdProp = this->GetChildItem(_item, "threshold");
    QtProperty *granularityProp = this->GetChildItem(_item, "granularity");

    msgs::ImageGeom *imageMessage = (msgs::ImageGeom*)(message);
    imageMessage->set_uri(
        this->variantManager->value(fileProp).toString().toStdString());
    imageMessage->set_scale(
        this->variantManager->value(scaleProp).toDouble());
    imageMessage->set_height(
        this->variantManager->value(heightProp).toDouble());
    imageMessage->set_threshold(
        this->variantManager->value(thresholdProp).toInt());
    imageMessage->set_granularity(
        this->variantManager->value(granularityProp).toInt());
  }
  else if (type == "heightmap")
  {
    QtProperty *sizeProp = this->GetChildItem(_item, "size");
    QtProperty *offsetProp = this->GetChildItem(_item, "offset");
    QtProperty *fileProp = this->GetChildItem(_item, "filename");

    double px, py, pz;
    msgs::HeightmapGeom *heightmapMessage = (msgs::HeightmapGeom*)(message);

    msgs::Set(heightmapMessage->mutable_image(),
        common::Image(this->variantManager->value(
            fileProp).toString().toStdString()));

    px = this->variantManager->value(
         this->GetChildItem(sizeProp, "x")).toDouble();
    py = this->variantManager->value(
         this->GetChildItem(sizeProp, "y")).toDouble();
    pz = this->variantManager->value(
         this->GetChildItem(sizeProp, "z")).toDouble();

    heightmapMessage->mutable_size()->set_x(px);
    heightmapMessage->mutable_size()->set_y(py);
    heightmapMessage->mutable_size()->set_z(pz);

    px = this->variantManager->value(
         this->GetChildItem(offsetProp, "x")).toDouble();
    py = this->variantManager->value(
         this->GetChildItem(offsetProp, "y")).toDouble();
    pz = this->variantManager->value(
         this->GetChildItem(offsetProp, "z")).toDouble();

    heightmapMessage->mutable_origin()->set_x(px);
    heightmapMessage->mutable_origin()->set_y(py);
    heightmapMessage->mutable_origin()->set_z(pz);
  }
  else if (type == "mesh")
  {
    QtProperty *sizeProp = this->GetChildItem(_item, "scale");
    QtProperty *fileProp = this->GetChildItem(_item, "filename");

    double px, py, pz;
    msgs::MeshGeom *meshMessage = (msgs::MeshGeom*)(message);
    meshMessage->set_filename(this->variantManager->value(
          fileProp).toString().toStdString());

    px = this->variantManager->value(
         this->GetChildItem(sizeProp, "x")).toDouble();
    py = this->variantManager->value(
         this->GetChildItem(sizeProp, "y")).toDouble();
    pz = this->variantManager->value(
         this->GetChildItem(sizeProp, "z")).toDouble();

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

  this->FillMsgField(this->GetChildItem(_item, "x"), posMessage, posReflection,
      posDescriptor->FindFieldByName("x"));
  this->FillMsgField(this->GetChildItem(_item, "y"), posMessage, posReflection,
      posDescriptor->FindFieldByName("y"));
  this->FillMsgField(this->GetChildItem(_item, "z"), posMessage, posReflection,
      posDescriptor->FindFieldByName("z"));

  double roll, pitch, yaw;
  roll = this->variantManager->value(
      this->GetChildItem(_item, "roll")).toDouble();
  pitch = this->variantManager->value(
      this->GetChildItem(_item, "pitch")).toDouble();
  yaw = this->variantManager->value(
      this->GetChildItem(_item, "yaw")).toDouble();
  math::Quaternion q(roll, pitch, yaw);

  orientReflection->SetDouble(
      orientMessage,
      orientDescriptor->FindFieldByName("x"),
      q.x);
  orientReflection->SetDouble(
      orientMessage,
      orientDescriptor->FindFieldByName("y"),
      q.y);
  orientReflection->SetDouble(
      orientMessage,
      orientDescriptor->FindFieldByName("z"),
      q.z);
  orientReflection->SetDouble(
      orientMessage,
      orientDescriptor->FindFieldByName("w"),
      q.w);
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
    QtProperty *nameItem = this->GetChildItem(_item, "name");
    QtVariantProperty *idItem =
        dynamic_cast<QtVariantProperty *>(this->GetChildItem(_item, "id"));
    ((msgs::Link*)(_message))->set_name(nameItem->valueText().toStdString());
    ((msgs::Link*)(_message))->set_id(idItem->value().toInt());
  }
  else if (_item->propertyName().toStdString() == "collision")
  {
    QtProperty *nameItem = this->GetChildItem(_item, "name");
    QtVariantProperty *idItem =
        dynamic_cast<QtVariantProperty *>(this->GetChildItem(_item, "id"));
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

  return NULL;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::GetParentItemValue(const std::string &_name)
{
  QtProperty *result = NULL;

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
      iter != properties.end(); ++iter)
  {
    if ((*iter)->valueText().toStdString() == _name)
      return NULL;
    else if ((result = this->GetParentItemValue(*iter, _name)) != NULL)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::GetParentItemValue(QtProperty *_item,
                                           const std::string &_name)
{
  if (!_item)
    return NULL;

  QtProperty *result = NULL;

  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((*iter)->valueText().toStdString() == _name)
    {
      result = _item;
      break;
    }
    else if ((result = this->GetParentItemValue(*iter, _name)) != NULL)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::GetParentItem(const std::string &_name)
{
  QtProperty *result = NULL;

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
      iter != properties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == _name)
      return NULL;
    else if ((result = this->GetParentItem(*iter, _name)) != NULL)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::GetParentItem(QtProperty *_item,
                                           const std::string &_name)
{
  if (!_item)
    return NULL;

  QtProperty *result = NULL;

  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((*iter)->propertyName().toStdString() == _name)
    {
      result = _item;
      break;
    }
    else if ((result = this->GetParentItem(*iter, _name)) != NULL)
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
QtProperty *ModelListWidget::GetChildItemValue(const std::string &_name)
{
  QtProperty *result = NULL;

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
      iter != properties.end(); ++iter)
  {
    if ((result = this->GetChildItemValue(*iter, _name)) != NULL)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::GetChildItemValue(QtProperty *_item,
                                          const std::string &_name)
{
  if (!_item)
    return NULL;
  if (_item->valueText().toStdString() == _name)
    return _item;

  QtProperty *result = NULL;
  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((result = this->GetChildItem(*iter, _name)) != NULL)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::GetChildItem(const std::string &_name)
{
  QtProperty *result = NULL;

  QList<QtProperty*> properties = this->propTreeBrowser->properties();
  for (QList<QtProperty*>::iterator iter = properties.begin();
      iter != properties.end(); ++iter)
  {
    if ((result = this->GetChildItem(*iter, _name)) != NULL)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
QtProperty *ModelListWidget::GetChildItem(QtProperty *_item,
                                          const std::string &_name)
{
  if (!_item)
    return NULL;
  if (_item->propertyName().toStdString() == _name)
    return _item;

  QtProperty *result = NULL;
  QList<QtProperty*> subProperties = _item->subProperties();
  for (QList<QtProperty*>::iterator iter = subProperties.begin();
      iter != subProperties.end(); ++iter)
  {
    if ((result = this->GetChildItem(*iter, _name)) != NULL)
      break;
  }

  return result;
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::SphericalCoordinates &_msg,
                                       QtProperty * /*_parent*/)
{
  QtVariantProperty *item = NULL;

  item = this->variantManager->addProperty(
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
  this->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  item = this->variantManager->addProperty(QVariant::Double, tr("Latitude"));
  item->setValue(_msg.latitude_deg());
  this->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  item = this->variantManager->addProperty(QVariant::Double, tr("Longitude"));
  item->setValue(_msg.longitude_deg());
  this->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  item = this->variantManager->addProperty(QVariant::Double, tr("Elevation"));
  item->setValue(_msg.elevation());
  this->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  item = this->variantManager->addProperty(QVariant::Double, tr("Heading"));
  item->setValue(_msg.heading_deg());
  this->propTreeBrowser->addProperty(item);
  item->setEnabled(false);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Joint &_msg,
                                       QtProperty * /*_parent*/)
{
  QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

  // joint name
  item = this->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  this->propTreeBrowser->addProperty(item);
  item->setEnabled(false);

  // joint type
  if (_msg.has_type())
  {
    item = this->variantManager->addProperty(QVariant::String,
                                             tr("type"));
    std::string jointType = msgs::ConvertJointType(_msg.type());
    item->setValue(jointType.c_str());
    this->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  // parent link
  if (_msg.has_parent())
  {
    item = this->variantManager->addProperty(QVariant::String,
                                               tr("parent link"));
    item->setValue(_msg.parent().c_str());
    this->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  // child link
  if (_msg.has_child())
  {
    item = this->variantManager->addProperty(QVariant::String,
                                               tr("child link"));
    item->setValue(_msg.child().c_str());
    this->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  // Pose value
  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  this->propTreeBrowser->addProperty(topItem);
  topItem->setEnabled(false);

  this->FillPoseProperty(_msg.pose(), topItem);

  // Angle
  for (int i = 0; i < _msg.angle_size(); ++i)
  {
    std::string angleName = "angle_" + boost::lexical_cast<std::string>(i);
    item = this->variantManager->addProperty(QVariant::String,
                                             QString::fromStdString(angleName));
    item->setValue(_msg.angle(i));
    this->propTreeBrowser->addProperty(item);
    item->setEnabled(false);
  }

  // Add joint axes if present
  for (int i = 0; i < 2; ++i)
  {
    const msgs::Axis *axis = NULL;
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
      topItem = this->variantManager->addProperty(
          QtVariantPropertyManager::groupTypeId(), tr(axisName.c_str()));
      this->propTreeBrowser->addProperty(topItem);
      topItem->setEnabled(false);

      /// XYZ of the axis
      QtProperty *xyzItem = this->variantManager->addProperty(
          QtVariantPropertyManager::groupTypeId(), tr("xyz"));
      topItem->addSubProperty(xyzItem);
      xyzItem->setEnabled(false);
      this->FillVector3dProperty(axis->xyz(), xyzItem);

      // lower limit
      item = this->variantManager->addProperty(QVariant::Double, tr("lower"));
      item->setValue(axis->limit_lower());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // upper limit
      item = this->variantManager->addProperty(QVariant::Double, tr("upper"));
      item->setValue(axis->limit_upper());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // limit effort
      item = this->variantManager->addProperty(QVariant::Double, tr("effort"));
      item->setValue(axis->limit_effort());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // limit velocity
      item = this->variantManager->addProperty(QVariant::Double,
                                               tr("velocity"));
      item->setValue(axis->limit_velocity());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // damping
      item = this->variantManager->addProperty(QVariant::Double, tr("damping"));
      item->setValue(axis->damping());
      topItem->addSubProperty(item);
      item->setEnabled(false);

      // friction
      item = this->variantManager->addProperty(QVariant::Double,
                                               tr("friction"));
      item->setValue(axis->friction());
      topItem->addSubProperty(item);
      item->setEnabled(false);
    }
  }
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Link &_msg,
                                       QtProperty *_parent)
{
  QtProperty *topItem = NULL;
  QtProperty *inertialItem = NULL;
  QtVariantProperty *item = NULL;

  // id, store it but but make it hidden
  QtBrowserItem *browserItem = NULL;
  item = this->variantManager->addProperty(QVariant::String, tr("id"));
  item->setValue(_msg.id());
  this->AddProperty(item, _parent);
  browserItem = this->propTreeBrowser->items(item)[0];
  this->propTreeBrowser->setItemVisible(browserItem, false);

  // name
  item = this->variantManager->addProperty(QVariant::String, tr("name"));
  item->setValue(_msg.name().c_str());
  this->AddProperty(item, _parent);
  // TODO: setting link name currently causes problems
  item->setEnabled(false);

  // Self-collide
  item = this->variantManager->addProperty(QVariant::Bool, tr("self_collide"));
  if (_msg.has_self_collide())
    item->setValue(_msg.self_collide());
  else
    item->setValue(true);
  this->AddProperty(item, _parent);
  item->setEnabled(false);

  // gravity
  item = this->variantManager->addProperty(QVariant::Bool, tr("gravity"));
  if (_msg.has_gravity())
    item->setValue(_msg.gravity());
  else
    item->setValue(true);
  this->AddProperty(item, _parent);

  // kinematic
  item = this->variantManager->addProperty(QVariant::Bool, tr("kinematic"));
  if (_msg.has_kinematic())
    item->setValue(_msg.kinematic());
  else
    item->setValue(false);
  this->AddProperty(item, _parent);

  // canonical
  item = this->variantManager->addProperty(QVariant::Bool, tr("canonical"));
  if (_msg.has_canonical())
    item->setValue(_msg.canonical());
  else
    item->setValue(false);
  this->AddProperty(item, _parent);
  item->setEnabled(false);

  // pose
  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  this->AddProperty(topItem, _parent);

  this->FillPoseProperty(_msg.pose(), topItem);
  if (_msg.has_canonical() && _msg.canonical())
    topItem->setEnabled(false);

  // Inertial
  inertialItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("inertial"));
  this->AddProperty(inertialItem, _parent);

  // TODO: disable setting inertial properties until there are tests
  // in place to verify the functionality
  inertialItem->setEnabled(false);

  // Inertial::Mass
  item = this->variantManager->addProperty(QVariant::Double, tr("mass"));
  if (_msg.inertial().has_mass())
    item->setValue(_msg.inertial().mass());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::LinearDamping
/*  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("linear_damping"));
  if (_msg.has_linear_damping())
    item->setValue(_msg.linear_damping());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::AngularDamping
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("angular_damping"));
  if (_msg.has_angular_damping())
    item->setValue(_msg.angular_damping());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);
*/
  // Inertial::ixx
  item = this->variantManager->addProperty(QVariant::Double, tr("ixx"));
  if (_msg.inertial().has_ixx())
    item->setValue(_msg.inertial().ixx());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::ixy
  item = this->variantManager->addProperty(QVariant::Double, tr("ixy"));
  if (_msg.inertial().has_ixy())
    item->setValue(_msg.inertial().ixy());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::ixz
  item = this->variantManager->addProperty(QVariant::Double, tr("ixz"));
  if (_msg.inertial().has_ixz())
    item->setValue(_msg.inertial().ixz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::iyy
  item = this->variantManager->addProperty(QVariant::Double, tr("iyy"));
  if (_msg.inertial().has_iyy())
    item->setValue(_msg.inertial().iyy());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::iyz
  item = this->variantManager->addProperty(QVariant::Double, tr("iyz"));
  if (_msg.inertial().has_iyz())
    item->setValue(_msg.inertial().iyz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::izz
  item = this->variantManager->addProperty(QVariant::Double, tr("izz"));
  if (_msg.inertial().has_izz())
    item->setValue(_msg.inertial().izz());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  inertialItem->addSubProperty(topItem);
  this->FillPoseProperty(_msg.inertial().pose(), topItem);

  for (int i = 0; i < _msg.collision_size(); i++)
  {
    QtVariantProperty *prop;
    prop = this->variantManager->addProperty(
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
    prop = this->variantManager->addProperty(
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
    prop = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), "sensor");
    prop->setToolTip(tr(_msg.sensor(i).name().c_str()));
    this->AddProperty(prop, _parent);

    // this->FillPropertyTree(_msg.sensor(i), prop);
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

  QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

  // id, store it but but make it hidden
  QtBrowserItem *browserItem = NULL;
  item = this->variantManager->addProperty(QVariant::String, tr("id"));
  item->setValue(_msg.id());
  _parent->addSubProperty(item);
  browserItem = this->propTreeBrowser->items(item)[0];
  this->propTreeBrowser->setItemVisible(browserItem, false);

  // name
  item = this->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  _parent->addSubProperty(item);

  // Laser Retro value
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("laser_retro"));
  if (_msg.has_laser_retro())
    item->setValue(_msg.laser_retro());
  else
    item->setValue(0.0);
  _parent->addSubProperty(item);

  // Pose value
  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("pose"));
  _parent->addSubProperty(topItem);
  this->FillPoseProperty(_msg.pose(), topItem);

  // Geometry shape value
  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("geometry"));
  _parent->addSubProperty(topItem);
  this->FillPropertyTree(_msg.geometry(), topItem);

  // Surface value
  topItem = this->variantManager->addProperty(
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

  QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

  // Restituion Coefficient
  item = this->variantManager->addProperty(QVariant::Double,
      tr("restitution_coefficient"));
  item->setValue(_msg.restitution_coefficient());
  _parent->addSubProperty(item);

  // Bounce Threshold
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("bounce_threshold"));
  item->setValue(_msg.bounce_threshold());
  _parent->addSubProperty(item);

  // Soft CFM
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("soft_cfm"));
  item->setValue(_msg.soft_cfm());
  _parent->addSubProperty(item);

  // Soft ERP
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("soft_erp"));
  item->setValue(_msg.soft_erp());
  _parent->addSubProperty(item);

  // KP
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("kp"));
  item->setValue(_msg.kp());
  _parent->addSubProperty(item);

  // KD
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("kd"));
  item->setValue(_msg.kd());
  _parent->addSubProperty(item);

  // max vel
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("max_vel"));
  item->setValue(_msg.max_vel());
  _parent->addSubProperty(item);

  // min depth
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("min_depth"));
  item->setValue(_msg.min_depth());
  _parent->addSubProperty(item);

  // Friction
  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("friction"));
  _parent->addSubProperty(topItem);

  // Mu
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("mu"));
  item->setValue(_msg.friction().mu());
  topItem->addSubProperty(item);

  // Mu2
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("mu2"));
  item->setValue(_msg.friction().mu2());
  topItem->addSubProperty(item);

  // slip1
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("slip1"));
  item->setValue(_msg.friction().slip1());
  topItem->addSubProperty(item);

  // slip2
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("slip2"));
  item->setValue(_msg.friction().slip2());
  topItem->addSubProperty(item);

  // Fdir1
  QtProperty *fdirItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("fdir1"));
    topItem->addSubProperty(fdirItem);
    this->FillVector3dProperty(_msg.friction().fdir1(), fdirItem);
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

  QtVariantProperty *item = NULL;

  item = this->variantManager->addProperty(
      QtVariantPropertyManager::enumTypeId(), tr("type"));
  QStringList types;
  types << "BOX" << "SPHERE" << "CYLINDER" << "PLANE" << "MESH" << "IMAGE"
        << "HEIGHTMAP";
  item->setAttribute("enumNames", types);
  _parent->addSubProperty(item);

  if (_msg.type() == msgs::Geometry::BOX)
  {
    item->setValue(0);
    QtProperty *sizeItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("size"));
    _parent->addSubProperty(sizeItem);
    this->FillVector3dProperty(_msg.box().size(), sizeItem);
  }
  else if (_msg.type() == msgs::Geometry::SPHERE)
  {
    item->setValue(1);

    item = this->variantManager->addProperty(QVariant::Double,
        tr("radius"));
    item->setValue(_msg.sphere().radius());
    _parent->addSubProperty(item);
  }
  else if (_msg.type() == msgs::Geometry::CYLINDER)
  {
    item->setValue(2);
    item = this->variantManager->addProperty(QVariant::Double,
        tr("radius"));
    item->setValue(_msg.cylinder().radius());
    _parent->addSubProperty(item);

    item = this->variantManager->addProperty(QVariant::Double,
        tr("length"));
    item->setValue(_msg.cylinder().length());
    _parent->addSubProperty(item);
  }
  else if (_msg.type() == msgs::Geometry::PLANE)
  {
    item->setValue(3);
    QtProperty *normalItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("normal"));
    _parent->addSubProperty(normalItem);
    this->FillVector3dProperty(_msg.plane().normal(), normalItem);
  }
  else if (_msg.type() == msgs::Geometry::MESH)
  {
    item->setValue(4);

    item = this->variantManager->addProperty(QVariant::String,
        tr("filename"));
    item->setValue(_msg.mesh().filename().c_str());
    _parent->addSubProperty(item);

    QtProperty *sizeItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("scale"));
    _parent->addSubProperty(sizeItem);
    this->FillVector3dProperty(_msg.mesh().scale(), sizeItem);
  }
  else if (_msg.type() == msgs::Geometry::IMAGE)
  {
    item->setValue(5);

    item = this->variantManager->addProperty(QVariant::String, tr("uri"));
    item->setValue(_msg.image().uri().c_str());
    _parent->addSubProperty(item);

    item = this->variantManager->addProperty(QVariant::Double,
        tr("scale"));
    item->setValue(_msg.image().scale());
    _parent->addSubProperty(item);

    item = this->variantManager->addProperty(QVariant::Double,
        tr("height"));
    item->setValue(_msg.image().height());
    _parent->addSubProperty(item);

    item = this->variantManager->addProperty(QVariant::Int,
        tr("threshold"));
    item->setValue(_msg.image().threshold());
    _parent->addSubProperty(item);

    item = this->variantManager->addProperty(QVariant::Int,
        tr("granularity"));
    item->setValue(_msg.image().granularity());
    _parent->addSubProperty(item);
  }
  else if (_msg.type() == msgs::Geometry::HEIGHTMAP)
  {
    item->setValue(6);

    item = this->variantManager->addProperty(QVariant::String, tr("uri"));
    item->setValue(_msg.image().uri().c_str());
    _parent->addSubProperty(item);

    QtProperty *sizeItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(),
        tr("size"));
    _parent->addSubProperty(sizeItem);
    this->FillVector3dProperty(_msg.heightmap().size(), sizeItem);

    sizeItem = this->variantManager->addProperty(
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

  QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

  // Name value
  item = this->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  _parent->addSubProperty(item);

  // Laser Retro value
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("laser_retro"));
  if (_msg.has_laser_retro())
    item->setValue(_msg.laser_retro());
  else
    item->setValue(0.0);
  _parent->addSubProperty(item);

  // cast shadows value
  item = this->variantManager->addProperty(QVariant::Bool,
                                           tr("cast_shadows"));
  if (_msg.has_cast_shadows())
    item->setValue(_msg.cast_shadows());
  else
    item->setValue(true);
  _parent->addSubProperty(item);

  // transparency
  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("transparency"));
  if (_msg.has_transparency())
    item->setValue(_msg.transparency());
  else
    item->setValue(0.0);
  _parent->addSubProperty(item);


  // Pose value
  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("pose"));
  _parent->addSubProperty(topItem);
  this->FillPoseProperty(_msg.pose(), topItem);

  // Geometry shape value
  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(),
      tr("geometry"));
  _parent->addSubProperty(topItem);
  this->FillPropertyTree(_msg.geometry(), topItem);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Model &_msg,
                                       QtProperty * /*_parent*/)
{
  QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

  item = this->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  this->propTreeBrowser->addProperty(item);
  // TODO: setting model name currently causes problems
  item->setEnabled(false);

  item = this->variantManager->addProperty(QVariant::Bool,
                                           tr("is_static"));
  if (_msg.has_is_static())
    item->setValue(_msg.is_static());
  else
    item->setValue(false);
  /// \todo Dynamically setting a model static doesn't currently work.
  item->setEnabled(false);
  this->propTreeBrowser->addProperty(item);

  item = this->variantManager->addProperty(QVariant::Bool, tr("self_collide"));
  if (_msg.has_self_collide())
    item->setValue(_msg.self_collide());
  else
    item->setValue(false);
  item->setEnabled(false);
  this->propTreeBrowser->addProperty(item);

  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  QtBrowserItem *bItem = this->propTreeBrowser->addProperty(topItem);
  this->propTreeBrowser->setExpanded(bItem, false);
  this->FillPoseProperty(_msg.pose(), topItem);

  for (int i = 0; i < _msg.link_size(); i++)
  {
    QtVariantProperty *prop;
    prop = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), tr("link"));
    prop->setToolTip(tr(_msg.link(i).name().c_str()));

    bItem = this->propTreeBrowser->addProperty(prop);
    this->propTreeBrowser->setExpanded(bItem, false);

    this->FillPropertyTree(_msg.link(i), prop);
  }
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
  math::Vector3 value;
  value = msgs::Convert(_msg);
  value.Round(6);

  // Add X value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "x"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "x");
    _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>
    (this->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  item->setValue(value.x);

  // Add Y value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "y"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "y");
    _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>(this->variantFactory->propertyManager(
    item))->setAttribute(item, "decimals", 6);
  item->setValue(value.y);

  // Add Z value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "z"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "z");
    _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>(this->variantFactory->propertyManager(
    item))->setAttribute(item, "decimals", 6);
  item->setValue(value.z);
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
  math::Pose value;
  value = msgs::Convert(_msg);
  value.Round(6);

  math::Vector3 rpy = value.rot.GetAsEuler();
  rpy.Round(6);

  this->FillVector3dProperty(_msg.position(), _parent);

  // Add Roll value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "roll"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "roll");
    _parent->addSubProperty(item);
    static_cast<QtVariantPropertyManager *>(
        this->variantFactory->propertyManager(
        item))->setAttribute(item, "decimals", 6);
    static_cast<QtVariantPropertyManager *>(
        this->variantFactory->propertyManager(
        item))->setAttribute(item, "singleStep", 0.05);
  }
  item->setValue(rpy.x);

  // Add Pitch value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "pitch"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "pitch");
    _parent->addSubProperty(item);
    static_cast<QtVariantPropertyManager *>(
        this->variantFactory->propertyManager(
        item))->setAttribute(item, "decimals", 6);
    static_cast<QtVariantPropertyManager *>(
        this->variantFactory->propertyManager(
        item))->setAttribute(item, "singleStep", 0.05);
  }
  item->setValue(rpy.y);

  // Add Yaw value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "yaw"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "yaw");
    _parent->addSubProperty(item);
    static_cast<QtVariantPropertyManager *>(
        this->variantFactory->propertyManager(
        item))->setAttribute(item, "decimals", 6);
    static_cast<QtVariantPropertyManager *>(
        this->variantFactory->propertyManager(
        item))->setAttribute(item, "singleStep", 0.05);
  }
  item->setValue(rpy.z);
}

/////////////////////////////////////////////////
void ModelListWidget::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    this->removeEntityList.push_back(_msg->data());
  }
}

/////////////////////////////////////////////////
void ModelListWidget::ProcessRemoveEntity()
{
  for (RemoveEntity_L::iterator iter = this->removeEntityList.begin();
       iter != this->removeEntityList.end(); ++iter)
  {
    this->RemoveEntity(*iter);
  }
  this->removeEntityList.clear();
}

/////////////////////////////////////////////////
void ModelListWidget::OnRemoveScene(const std::string &/*_name*/)
{
  this->ResetTree();
  this->propTreeBrowser->clear();
  if (this->node)
    this->node->Fini();
  this->node.reset();

  this->requestPub.reset();
  this->modelPub.reset();
  this->scenePub.reset();
  this->physicsPub.reset();
  this->lightPub.reset();
  this->responseSub.reset();
  this->requestSub.reset();
}

/////////////////////////////////////////////////
void ModelListWidget::OnCreateScene(const std::string &_name)
{
  this->ResetTree();

  this->propTreeBrowser->clear();
  this->InitTransport(_name);

  // this->requestMsg = msgs::CreateRequest("scene_info");
  // this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelListWidget::InitTransport(const std::string &_name)
{
  if (this->node)
  {
    this->node->Fini();
    this->node.reset();
    this->requestPub.reset();
    this->modelPub.reset();
    this->scenePub.reset();
    this->physicsPub.reset();
    this->lightPub.reset();
    this->responseSub.reset();
    this->requestSub.reset();
  }

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_name);

  this->modelPub = this->node->Advertise<msgs::Model>("~/model/modify");
  this->scenePub = this->node->Advertise<msgs::Scene>("~/scene");
  this->physicsPub = this->node->Advertise<msgs::Physics>("~/physics");

  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->responseSub = this->node->Subscribe("~/response",
                                            &ModelListWidget::OnResponse, this);

  this->requestSub = this->node->Subscribe("~/request",
      &ModelListWidget::OnRequest, this, false);
}

/////////////////////////////////////////////////
void ModelListWidget::ResetTree()
{
  this->modelTreeWidget->clear();

  // Create the top level of items in the tree widget
  {
    // GUI item
    this->guiItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("GUI"))));
    this->guiItem->setData(0, Qt::UserRole, QVariant(tr("GUI")));
    this->modelTreeWidget->addTopLevelItem(this->guiItem);

    // Scene item
    this->ResetScene();

    // Spherical coordinates item
    this->sphericalCoordItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Spherical Coordinates"))));
    this->sphericalCoordItem->setData(0,
        Qt::UserRole, QVariant(tr("Spherical Coordinates")));
    this->modelTreeWidget->addTopLevelItem(this->sphericalCoordItem);

    // Physics item
    this->physicsItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Physics"))));
    this->physicsItem->setData(0, Qt::UserRole, QVariant(tr("Physics")));
    this->modelTreeWidget->addTopLevelItem(this->physicsItem);

    // Models item
    this->modelsItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Models"))));
    this->modelsItem->setData(0, Qt::UserRole, QVariant(tr("Models")));
    this->modelTreeWidget->addTopLevelItem(this->modelsItem);

    // Lights item
    this->lightsItem = new QTreeWidgetItem(
        static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("%1").arg(tr("Lights"))));
    this->lightsItem->setData(0, Qt::UserRole, QVariant(tr("Lights")));
    this->modelTreeWidget->addTopLevelItem(this->lightsItem);
  }

  this->fillingPropertyTree = false;
  this->selectedProperty = NULL;
}

/////////////////////////////////////////////////
ModelListSheetDelegate::ModelListSheetDelegate(QTreeView *view, QWidget *parent)
    : QItemDelegate(parent), m_view(view)
{
}

/////////////////////////////////////////////////
void ModelListSheetDelegate::paint(QPainter *painter,
    const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  const QAbstractItemModel *model = index.model();
  Q_ASSERT(model);

  if (!model->parent(index).isValid())
  {
    QRect r = option.rect;
    static const int i = 9;

    // draw text
    QRect textrect = QRect(r.left()+4, r.top(),
        r.width() - ((5*i)/2), r.height());
    QString text = elidedText(option.fontMetrics,
        textrect.width(),
        Qt::ElideMiddle,
        model->data(index, Qt::DisplayRole).toString());

    if (option.state & QStyle::State_Selected)
      painter->setPen(QPen(QColor(245, 129, 19), 1));

    m_view->style()->drawItemText(painter, textrect, Qt::AlignLeft,
        option.palette, m_view->isEnabled(), text);
  }
  else
  {
    QItemDelegate::paint(painter, option, index);
  }
}

/////////////////////////////////////////////////
QSize ModelListSheetDelegate::sizeHint(const QStyleOptionViewItem &opt,
                                               const QModelIndex &index) const
{
  QStyleOptionViewItem option = opt;
  QSize sz = QItemDelegate::sizeHint(opt, index) + QSize(2, 2);
  return sz;
}

/////////////////////////////////////////////////
void ModelListWidget::ResetScene()
{
  this->sceneItem = new QTreeWidgetItem(
      static_cast<QTreeWidgetItem*>(0),
      QStringList(QString("%1").arg(tr("Scene"))));
  this->sceneItem->setData(0, Qt::UserRole, QVariant(tr("Scene")));
  this->modelTreeWidget->addTopLevelItem(this->sceneItem);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Scene &_msg,
                                       QtProperty * /*_parent*/)
{
  // QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

  // Create and set the ambient color property
  item = this->variantManager->addProperty(QVariant::Color, tr("ambient"));
  if (_msg.has_ambient())
  {
    QColor clr(_msg.ambient().r()*255, _msg.ambient().g()*255,
               _msg.ambient().b()*255, _msg.ambient().a()*255);
    item->setValue(clr);
  }
  this->propTreeBrowser->addProperty(item);

  // Create and set the background color property
  item = this->variantManager->addProperty(QVariant::Color, tr("background"));
  if (_msg.has_background())
  {
    QColor clr(_msg.background().r()*255, _msg.background().g()*255,
               _msg.background().b()*255, _msg.background().a()*255);
    item->setValue(clr);
  }
  this->propTreeBrowser->addProperty(item);

  // Create and set the shadows property
  item = this->variantManager->addProperty(QVariant::Bool, tr("shadows"));
  if (_msg.has_shadows())
    item->setValue(_msg.shadows());
  this->propTreeBrowser->addProperty(item);

  /// \TODO: Put fog back in
  /*topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("fog"));
  QtBrowserItem *bItem = this->propTreeBrowser->addProperty(topItem);
  this->propTreeBrowser->setExpanded(bItem, false);

  item = this->variantManager->addProperty(QVariant::Color, tr("color"));
  topItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double, tr("start"));
  topItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double, tr("end"));
  topItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double, tr("density"));
  topItem->addSubProperty(item);
  */

  /// \TODO: Put sky modification back in GUI
  /*
  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("sky"));
  bItem = this->propTreeBrowser->addProperty(topItem);
  this->propTreeBrowser->setExpanded(bItem, false);
  */
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Physics &_msg,
                                       QtProperty * /*_parent*/)
{
  QtVariantProperty *item = NULL;

  if (_msg.has_type())
    this->physicsType = _msg.type();

  item = this->variantManager->addProperty(QVariant::Bool,
    tr("enable physics"));
  if (_msg.has_enable_physics())
    item->setValue(_msg.enable_physics());
  this->propTreeBrowser->addProperty(item);

  item = this->variantManager->addProperty(QVariant::Double,
      tr("real time update rate"));
  static_cast<QtVariantPropertyManager*>
    (this->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_real_time_update_rate())
    item->setValue(_msg.real_time_update_rate());
  this->propTreeBrowser->addProperty(item);

  item = this->variantManager->addProperty(QVariant::Double,
      tr("max step size"));
  static_cast<QtVariantPropertyManager*>
    (this->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_max_step_size())
    item->setValue(_msg.max_step_size());
  this->propTreeBrowser->addProperty(item);

  QtProperty *gravityItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("gravity"));
  this->propTreeBrowser->addProperty(gravityItem);
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

  QtProperty *magneticFieldItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("magnetic field"));
  this->propTreeBrowser->addProperty(magneticFieldItem);
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

  QtProperty *solverItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("solver"));
  this->propTreeBrowser->addProperty(solverItem);

  item = this->variantManager->addProperty(QVariant::Int, tr("iterations"));
  if (_msg.has_iters())
    item->setValue(_msg.iters());
  solverItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double, tr("SOR"));
  static_cast<QtVariantPropertyManager*>
    (this->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_sor())
    item->setValue(_msg.sor());
  solverItem->addSubProperty(item);


  QtProperty *constraintsItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("constraints"));
  this->propTreeBrowser->addProperty(constraintsItem);

  item = this->variantManager->addProperty(QVariant::Double, tr("CFM"));
  static_cast<QtVariantPropertyManager*>
    (this->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_cfm())
    item->setValue(_msg.cfm());
  constraintsItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double, tr("ERP"));
  static_cast<QtVariantPropertyManager*>
    (this->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_erp())
    item->setValue(_msg.erp());
  constraintsItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("max velocity"));
  static_cast<QtVariantPropertyManager*>
    (this->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_contact_max_correcting_vel())
    item->setValue(_msg.contact_max_correcting_vel());
  constraintsItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double,
                                           tr("surface layer"));
  static_cast<QtVariantPropertyManager*>
    (this->variantFactory->propertyManager(item))->setAttribute(
        item, "decimals", 6);
  if (_msg.has_contact_surface_layer())
    item->setValue(_msg.contact_surface_layer());
  constraintsItem->addSubProperty(item);
}

/////////////////////////////////////////////////
void ModelListWidget::FillPropertyTree(const msgs::Light &_msg,
                                       QtProperty * /*_parent*/)
{
  QtVariantProperty *item = NULL;
  QtProperty *topItem = NULL;

  this->lightType = _msg.type();

  item = this->variantManager->addProperty(QVariant::String, tr("name"));
  if (_msg.has_name())
    item->setValue(_msg.name().c_str());
  this->propTreeBrowser->addProperty(item);

  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  this->propTreeBrowser->addProperty(topItem);
  if (_msg.has_pose())
    this->FillPoseProperty(_msg.pose(), topItem);
  else
    this->FillPoseProperty(msgs::Convert(math::Pose()), topItem);

  // Create and set the diffuse color property
  item = this->variantManager->addProperty(QVariant::Color, tr("diffuse"));
  if (_msg.has_diffuse())
  {
    QColor clr(_msg.diffuse().r()*255, _msg.diffuse().g()*255,
               _msg.diffuse().b()*255, _msg.diffuse().a()*255);
    item->setValue(clr);
  }
  this->propTreeBrowser->addProperty(item);

  // Create and set the specular color property
  item = this->variantManager->addProperty(QVariant::Color, tr("specular"));
  if (_msg.has_specular())
  {
    QColor clr(_msg.specular().r()*255, _msg.specular().g()*255,
               _msg.specular().b()*255, _msg.specular().a()*255);
    item->setValue(clr);
  }
  this->propTreeBrowser->addProperty(item);

  item = this->variantManager->addProperty(QVariant::Double, tr("range"));
  if (_msg.has_range())
    item->setValue(_msg.range());
  this->propTreeBrowser->addProperty(item);

  QtProperty *attenuationItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("attenuation"));
  this->propTreeBrowser->addProperty(attenuationItem);

  item = this->variantManager->addProperty(QVariant::Double, tr("constant"));
  if (_msg.has_attenuation_constant())
    item->setValue(_msg.attenuation_constant());
  attenuationItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double, tr("linear"));
  if (_msg.has_attenuation_linear())
    item->setValue(_msg.attenuation_linear());
  attenuationItem->addSubProperty(item);

  item = this->variantManager->addProperty(QVariant::Double, tr("quadratic"));
  if (_msg.has_attenuation_quadratic())
    item->setValue(_msg.attenuation_quadratic());
  attenuationItem->addSubProperty(item);

  if (_msg.has_spot_inner_angle() || _msg.has_spot_outer_angle() ||
      _msg.has_spot_falloff())
  {
    QtProperty *spotItem = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), tr("spot light"));
    this->propTreeBrowser->addProperty(spotItem);

    item = this->variantManager->addProperty(QVariant::Double,
                                             tr("inner angle"));
    if (_msg.has_spot_inner_angle())
      item->setValue(_msg.spot_inner_angle());
    spotItem->addSubProperty(item);

    item = this->variantManager->addProperty(QVariant::Double,
                                             tr("outer angle"));
    if (_msg.has_spot_outer_angle())
      item->setValue(_msg.spot_outer_angle());
    spotItem->addSubProperty(item);

    item = this->variantManager->addProperty(QVariant::Double,
                                             tr("falloff"));
    if (_msg.has_spot_falloff())
      item->setValue(_msg.spot_falloff());
    spotItem->addSubProperty(item);
  }
}

/////////////////////////////////////////////////
void ModelListWidget::ProcessLightMsgs()
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);

  for (LightMsgs_L::iterator iter = this->lightMsgs.begin();
       iter != this->lightMsgs.end(); ++iter)
  {
    std::string name = (*iter).name();

    QTreeWidgetItem *listItem = this->GetListItem((*iter).name(),
                                                  this->lightsItem);

    if (!listItem)
    {
      // Create a top-level tree item for the path
      QTreeWidgetItem *item = new QTreeWidgetItem(this->lightsItem,
          QStringList(QString("%1").arg(QString::fromStdString(name))));

      item->setData(0, Qt::UserRole, QVariant((*iter).name().c_str()));
      this->modelTreeWidget->addTopLevelItem(item);
    }
    else
    {
      listItem->setData(0, Qt::UserRole, QVariant((*iter).name().c_str()));
    }
  }
  this->lightMsgs.clear();
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
    this->propTreeBrowser->addProperty(_item);
}
