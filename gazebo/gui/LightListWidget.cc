/*
 * Copyright 2011 Nate Koenig
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
#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "sdf/sdf.hh"
#include "common/Image.hh"
#include "common/SystemPaths.hh"
#include "common/Console.hh"
#include "common/Events.hh"

#include "rendering/RenderEvents.hh"
#include "rendering/Rendering.hh"
#include "rendering/Scene.hh"
#include "rendering/UserCamera.hh"
#include "rendering/Visual.hh"
#include "gui/Gui.hh"

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "math/Angle.hh"
#include "math/Helpers.hh"

#include "gui/LightRightMenu.hh"
#include "gui/GuiEvents.hh"
#include "gui/qtpropertybrowser/qttreepropertybrowser.h"
#include "gui/qtpropertybrowser/qtvariantproperty.h"
#include "gui/LightListWidget.hh"

using namespace gazebo;
using namespace gui;

extern LightRightMenu *g_lightRightMenu;

/////////////////////////////////////////////////
LightListWidget::LightListWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("modelList");

  this->requestMsg = NULL;
  this->propMutex = new boost::mutex();
  this->receiveMutex = new boost::mutex();
  this->fillPropertyTree = false;

  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->lightListWidget = new QListWidget();
  this->lightListWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this->lightListWidget, SIGNAL(itemClicked(QListWidgetItem *)),
          this, SLOT(OnLightSelection(QListWidgetItem *)));
  connect(this->lightListWidget,
       SIGNAL(customContextMenuRequested(const QPoint &)),
       this, SLOT(OnCustomContextMenu(const QPoint &)));

/*  this->variantManager = new QtVariantPropertyManager();
  this->propTreeBrowser = new QtTreePropertyBrowser();
  this->variantFactory = new QtVariantEditorFactory();
  this->propTreeBrowser->setFactoryForManager(this->variantManager,
                                              this->variantFactory);
  connect(this->variantManager,
          SIGNAL(propertyChanged(QtProperty*)),
          this, SLOT(OnPropertyChanged(QtProperty *)));
  connect(this->propTreeBrowser,
          SIGNAL(currentItemChanged(QtBrowserItem*)),
          this, SLOT(OnCurrentPropertyChanged(QtBrowserItem *)));
          */


  // mainLayout->addWidget(this->lightListWidget, 0);
  // mainLayout->addWidget(this->propTreeBrowser, 1);

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->addWidget(this->lightListWidget, 0);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);

  mainLayout->addWidget(frame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->InitTransport();

  this->fillingPropertyTree = false;
  this->selectedProperty = NULL;

  /*this->connections.push_back(
      rendering::Events::ConnectCreateScene(
        boost::bind(&LightListWidget::OnCreateScene, this, _1)));

  this->connections.push_back(
      rendering::Events::ConnectRemoveScene(
        boost::bind(&LightListWidget::OnRemoveScene, this, _1)));

  this->connections.push_back(
      event::Events::ConnectSetSelectedEntity(
        boost::bind(&LightListWidget::OnSetSelectedEntity, this, _1)));
        */

  QTimer::singleShot(500, this, SLOT(Update()));
}

/////////////////////////////////////////////////
LightListWidget::~LightListWidget()
{
  delete this->propMutex;
  delete this->receiveMutex;
}

/////////////////////////////////////////////////
void LightListWidget::OnLightSelection(QListWidgetItem *_item)
{
  if (_item)
  {
    std::string name = _item->data(Qt::UserRole).toString().toStdString();
    event::Events::setSelectedEntity(name);
  }
  else
    this->selectedModelName.clear();
}

/////////////////////////////////////////////////
void LightListWidget::OnSetSelectedEntity(const std::string &/*_name*/)
{
  /*
  this->selectedModelName = _name;

  this->propTreeBrowser->clear();
  if (!this->selectedModelName.empty())
  {
    this->requestMsg = msgs::CreateRequest("entity_info",
        this->selectedModelName);
    this->requestPub->Publish(*this->requestMsg);
  }
  */
}

/////////////////////////////////////////////////
void LightListWidget::Update()
{
  /*
  if (this->fillPropertyTree)
  {
    this->receiveMutex->lock();
    this->poseMsgs.clear();
    this->fillingPropertyTree = true;
    this->FillPropertyTree(this->modelMsg, NULL);
    this->fillingPropertyTree = false;
    this->fillPropertyTree = false;
    this->receiveMutex->unlock();
  }
  */

  this->ProcessLightMsgs();
  // this->ProcessPoseMsgs();
  QTimer::singleShot(1000, this, SLOT(Update()));
}

/////////////////////////////////////////////////
void LightListWidget::OnLightMsg(ConstLightPtr &_msg)
{
  boost::mutex::scoped_lock lock(*this->receiveMutex);
  this->lightMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void LightListWidget::ProcessLightMsgs()
{
  this->receiveMutex->lock();

  for (LightMsgs_L::iterator iter = this->lightMsgs.begin();
       iter != this->lightMsgs.end(); ++iter)
  {
    std::string name = (*iter)->name();

    QListWidgetItem *listItem = this->GetLightListItem((*iter)->name());

    if (!listItem)
    {
      // Create a top-level tree item for the path
      QListWidgetItem *item = new QTreeWidgetItem(this->lightsItem,
          QString::fromStdString(name));

      item->setData(0, Qt::UserRole, QVariant((*iter)->name().c_str()));
      this->modelTreeWidget->addTopLevelItem(item);
    }
    else
    {
      listItem->setData(Qt::UserRole, QVariant((*iter)->name().c_str()));
    }
  }
  this->lightMsgs.clear();

  this->receiveMutex->unlock();
}

/////////////////////////////////////////////////
void LightListWidget::OnRemoveScene(const std::string &/*_name*/)
{
  this->poseMsgs.clear();
  this->lightListWidget->clear();
  this->propTreeBrowser->clear();
  this->node->Fini();
  this->node.reset();

  this->requestPub.reset();
  this->modelPub.reset();
  this->responseSub.reset();
  this->requestSub.reset();
  this->poseSub.reset();
}

/////////////////////////////////////////////////
void LightListWidget::OnCreateScene(const std::string &_name)
{
  this->lightListWidget->clear();
  this->propTreeBrowser->clear();

  this->InitTransport(_name);
}

/////////////////////////////////////////////////
void LightListWidget::InitTransport(const std::string &_name)
{
  if (this->node)
  {
    this->node->Fini();
    this->node.reset();
    this->requestPub.reset();
    this->responseSub.reset();
    this->requestSub.reset();
    // this->poseSub.reset();
  }


  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_name);

  this->lightSub = this->node->Subscribe("~/light",
      &LightListWidget::OnLightMsg, this);

  // this->requestPub =
  //     this->node->Advertise<msgs::Request>("~/request", 5, true);
  // this->responseSub = this->node->Subscribe("~/response",
  //     &LightListWidget::OnResponse, this);

  // this->requestSub = this->node->Subscribe("~/request",
  //     &LightListWidget::OnRequest, this);

  // this->poseSub = this->node->Subscribe("~/pose/info",
  //     &LightListWidget::OnPose, this);
}

/////////////////////////////////////////////////
QListWidgetItem *LightListWidget::GetLightListItem(const std::string &_name)
{
  QListWidgetItem *listItem = NULL;

  // Find an existing element with the name from the message
  for (int i = 0; i < this->lightListWidget->count() && !listItem; ++i)
  {
    QListWidgetItem *item = this->lightListWidget->item(i);
    std::string listData = item->data(Qt::UserRole).toString().toStdString();
    if (listData == _name)
    {
      listItem = item;
      break;
    }
  }

  return listItem;
}


/*
/////////////////////////////////////////////////
void LightListWidget::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  if (_msg->has_type() && _msg->type() == this->modelMsg.GetTypeName())
  {
    this->propMutex->lock();
    this->modelMsg.ParseFromString(_msg->serialized_data());
    this->propTreeBrowser->clear();
    this->fillPropertyTree = true;
    this->propMutex->unlock();
  }
  else if (_msg->has_type() && _msg->type() == "error")
  {
    if (_msg->response() == "nonexistant")
    {
      this->RemoveEntity(this->selectedModelName);
    }
  }

  delete this->requestMsg;
  this->requestMsg = NULL;
}
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::RemoveEntity(const std::string &_name)
{
  if (gui::has_entity_name(_name))
  {
    QListWidgetItem *listItem =
      this->GetModelListItem(gui::get_entity_id(_name));
    if (listItem)
    {
      int i = this->lightListWidget->indexOfTopLevelItem(listItem);
      this->lightListWidget->takeTopLevelItem(i);

      this->propTreeBrowser->clear();
      this->selectedModelName.clear();
      this->sdfElement.reset();
      this->fillPropertyTree = false;
    }
  }
}
  */

/////////////////////////////////////////////////
void LightListWidget::OnCustomContextMenu(const QPoint &_pt)
{
  QListWidgetItem *item = this->lightListWidget->itemAt(_pt);

  if (item)
  {
    g_lightRightMenu->Run(item->text().toStdString(),
                          this->lightListWidget->mapToGlobal(_pt));
  }
}

  /*
/////////////////////////////////////////////////
void LightListWidget::OnCurrentPropertyChanged(QtBrowserItem *_item)
{
  if (_item)
    this->selectedProperty = _item->property();
  else
    this->selectedProperty = NULL;
}
    */

  /*
/////////////////////////////////////////////////
void LightListWidget::OnPropertyChanged(QtProperty *_item)
{
  if (!this->propMutex->try_lock())
    return;

  if (this->selectedProperty != _item || this->fillingPropertyTree)
  {
    this->propMutex->unlock();
    return;
  }

  msgs::Model msg;

  msg.set_id(this->modelMsg.id());
  msg.set_name(this->modelMsg.name());

  const google::protobuf::Descriptor *descriptor = msg.GetDescriptor();
  const google::protobuf::Reflection *reflection = msg.GetReflection();

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
        this->FillMsg((*iter), reflection->AddMessage(&msg, field),
            field->message_type(), _item);
      }
      else
      {
        this->FillMsg((*iter),
            reflection->MutableMessage(&msg, field),
            field->message_type(), _item);
      }
    }
    else if (field)
    {
      this->FillMsgField((*iter), &msg, reflection, field);
    }
    else
    {
      gzerr << "Unable to process["
            << (*iter)->propertyName().toStdString() << "]\n";
    }
  }

  this->modelPub->Publish(msg);

  this->propMutex->unlock();
}
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillMsgField(QtProperty *_item,
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
    */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillGeometryMsg(QtProperty *_item,
    google::protobuf::Message *_message,
    const google::protobuf::Descriptor *_descriptor,
    QtProperty *_changedItem)
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
    imageMessage->set_filename(
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
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillPoseMsg(QtProperty *_item,
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
      */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillMsg(QtProperty *_item,
    google::protobuf::Message *_message,
    const google::protobuf::Descriptor *_descriptor,
    QtProperty *_changedItem)
{
  if (!_item)
    return;

  if (_item->propertyName().toStdString() == "link")
  {
    QtProperty *nameItem = this->GetChildItem(_item, "name");
    ((msgs::Link*)(_message))->set_name(nameItem->valueText().toStdString());
    ((msgs::Link*)(_message))->set_id(
      gui::get_entity_id(nameItem->valueText().toStdString()));
  }
  else if (_item->propertyName().toStdString() == "collision")
  {
    QtProperty *nameItem = this->GetChildItem(_item, "name");
    ((msgs::Collision*)_message)->set_name(nameItem->valueText().toStdString());
    ((msgs::Collision*)_message)->set_id(
      gui::get_entity_id(nameItem->valueText().toStdString()));
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
*/

  /*
/////////////////////////////////////////////////
QtProperty *LightListWidget::PopChildItem(QList<QtProperty*> &_list,
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
  */

  /*
/////////////////////////////////////////////////
QtProperty *LightListWidget::GetParentItemValue(const std::string &_name)
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
  */

  /*
/////////////////////////////////////////////////
QtProperty *LightListWidget::GetParentItemValue(QtProperty *_item,
                                           const std::string &_name)
{
  return NULL;
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
  */

  /*
/////////////////////////////////////////////////
QtProperty *LightListWidget::GetParentItem(const std::string &_name)
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
  */

  /*
/////////////////////////////////////////////////
QtProperty *LightListWidget::GetParentItem(QtProperty *_item,
                                           const std::string &_name)
{
  return NULL;
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
  */

  /*
/////////////////////////////////////////////////
bool LightListWidget::HasChildItem(QtProperty *_parent, QtProperty *_child)
{
  return false;
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
  */

  /*
/////////////////////////////////////////////////
QtProperty *LightListWidget::GetChildItemValue(const std::string &_name)
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
}*/

/////////////////////////////////////////////////
/*
QtProperty *LightListWidget::GetChildItemValue(QtProperty *_item,
                                          const std::string &_name)
{
  return NULL;
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
  */

  /*
/////////////////////////////////////////////////
QtProperty *LightListWidget::GetChildItem(const std::string &_name)
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
  */

  /*
/////////////////////////////////////////////////
QtProperty *LightListWidget::GetChildItem(QtProperty *_item,
                                          const std::string &_name)
{
  return NULL;
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
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillPropertyTree(const msgs::Link &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
    return;

  QtProperty *topItem = NULL;
  QtProperty *inertialItem = NULL;
  QtVariantProperty *item = NULL;

  item = this->variantManager->addProperty(QVariant::String, tr("name"));
  item->setValue(_msg.name().c_str());
  _parent->addSubProperty(item);

  // Self-collide
  item = this->variantManager->addProperty(QVariant::Bool, tr("self_collide"));
  if (_msg.has_self_collide())
    item->setValue(_msg.self_collide());
  else
    item->setValue(true);
  _parent->addSubProperty(item);

  // gravity
  item = this->variantManager->addProperty(QVariant::Bool, tr("gravity"));
  if (_msg.has_gravity())
    item->setValue(_msg.gravity());
  else
    item->setValue(true);
  _parent->addSubProperty(item);

  // kinematic
  item = this->variantManager->addProperty(QVariant::Bool, tr("kinematic"));
  if (_msg.has_kinematic())
    item->setValue(_msg.kinematic());
  else
    item->setValue(false);
  _parent->addSubProperty(item);

  topItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("pose"));
  _parent->addSubProperty(topItem);
  this->FillPoseProperty(_msg.pose(), topItem);

  // Inertial
  inertialItem = this->variantManager->addProperty(
      QtVariantPropertyManager::groupTypeId(), tr("inertial"));
  _parent->addSubProperty(inertialItem);

  // Inertial::Mass
  item = this->variantManager->addProperty(QVariant::Double, tr("mass"));
  if (_msg.inertial().has_mass())
    item->setValue(_msg.inertial().mass());
  else
    item->setValue(0.0);
  inertialItem->addSubProperty(item);

  // Inertial::LinearDamping
//  item = this->variantManager->addProperty(QVariant::Double,
//                                           tr("linear_damping"));
//  if (_msg.has_linear_damping())
//    item->setValue(_msg.linear_damping());
//  else
//    item->setValue(0.0);
//  inertialItem->addSubProperty(item);
//
//  // Inertial::AngularDamping
//  item = this->variantManager->addProperty(QVariant::Double,
//                                           tr("angular_damping"));
//  if (_msg.has_angular_damping())
//    item->setValue(_msg.angular_damping());
//  else
//    item->setValue(0.0);
//  inertialItem->addSubProperty(item);

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
    _parent->addSubProperty(prop);

    this->FillPropertyTree(_msg.collision(i), prop);
  }

  for (int i = 0; i < _msg.visual_size(); i++)
  {
    QtVariantProperty *prop;
    prop = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), tr("visual"));
    prop->setToolTip(tr(_msg.visual(i).name().c_str()));
    _parent->addSubProperty(prop);

    this->FillPropertyTree(_msg.visual(i), prop);
  }

  for (int i = 0; i < _msg.sensor_size(); i++)
  {
    QtVariantProperty *prop;
    prop = this->variantManager->addProperty(
        QtVariantPropertyManager::groupTypeId(), "sensor");
    prop->setToolTip(tr(_msg.sensor(i).name().c_str()));
    _parent->addSubProperty(prop);
    // this->FillPropertyTree(_msg.sensor(i), prop);
  }
}
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillPropertyTree(const msgs::Collision &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
    return;

  QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

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
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillPropertyTree(const msgs::Surface &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
    return;

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
    */
  /*

/////////////////////////////////////////////////
void LightListWidget::FillPropertyTree(const msgs::Geometry &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
    return;

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

    item = this->variantManager->addProperty(QVariant::String,
        tr("filename"));
    item->setValue(_msg.image().filename().c_str());
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

    item = this->variantManager->addProperty(QVariant::String,
        tr("filename"));
    item->setValue(_msg.image().filename().c_str());
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
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillPropertyTree(const msgs::Visual &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
    return;

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
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillPropertyTree(const msgs::Model &_msg,
                                       QtProperty *_parent)
{
  QtProperty *topItem = NULL;
  QtVariantProperty *item = NULL;

  item = this->variantManager->addProperty(QVariant::String,
                                           tr("name"));
  item->setValue(_msg.name().c_str());
  this->propTreeBrowser->addProperty(item);

  item = this->variantManager->addProperty(QVariant::Bool,
                                           tr("is_static"));
  if (_msg.has_is_static())
    item->setValue(_msg.is_static());
  else
    item->setValue(false);
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
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillVector3dProperty(const msgs::Vector3d &_msg,
                                           QtProperty *_parent)
{
  if (!_parent)
    return;

  QtVariantProperty *item;
  math::Vector3 value;
  value = msgs::Convert(_msg);
  value.Round(6);

  // Add X value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "x"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "x");
    if (_parent)
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
    if (_parent)
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
    if (_parent)
      _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>(this->variantFactory->propertyManager(
    item))->setAttribute(item, "decimals", 6);
  item->setValue(value.z);
}
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::FillPoseProperty(const msgs::Pose &_msg,
                                       QtProperty *_parent)
{
  if (!_parent)
    return;

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
    if (_parent)
    {
      _parent->addSubProperty(item);
    }
  }
  static_cast<QtVariantPropertyManager*>(this->variantFactory->propertyManager(
    item))->setAttribute(item, "decimals", 6);
  item->setValue(GZ_RTOD(rpy.x));

  // Add Pitch value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "pitch"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "pitch");
    if (_parent)
      _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>(this->variantFactory->propertyManager(
    item))->setAttribute(item, "decimals", 6);
  item->setValue(GZ_RTOD(rpy.y));

  // Add Yaw value
  item = static_cast<QtVariantProperty*>(this->GetChildItem(_parent, "yaw"));
  if (!item)
  {
    item = this->variantManager->addProperty(QVariant::Double, "yaw");
    if (_parent)
      _parent->addSubProperty(item);
  }
  static_cast<QtVariantPropertyManager*>(this->variantFactory->propertyManager(
    item))->setAttribute(item, "decimals", 6);
  item->setValue(GZ_RTOD(rpy.z));
}
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::ProcessPoseMsgs()
{
  this->receiveMutex->lock();
  this->propMutex->lock();
  this->fillingPropertyTree = true;

  PoseMsgs_L::iterator iter;
  for (iter = this->poseMsgs.begin(); iter != this->poseMsgs.end(); ++iter)
  {
    if ((*iter)->name().find(this->selectedModelName) != std::string::npos)
    {
      QtProperty *poseItem;
      QtProperty *nameItem = this->GetParentItemValue((*iter)->name());
      if (!nameItem)
        poseItem = this->GetChildItem("pose");
      else
        poseItem = this->GetChildItem(nameItem, "pose");
      this->FillPoseProperty(**iter, poseItem);
    }
  }
  this->poseMsgs.clear();

  this->fillingPropertyTree = false;
  this->propMutex->unlock();
  this->receiveMutex->unlock();
}
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::OnPose(ConstPosePtr &_msg)
{
  this->receiveMutex->lock();
  if (!this->selectedModelName.empty() &&
      _msg->name().find(this->selectedModelName) != std::string::npos)
  {
    PoseMsgs_L::iterator iter;

    // Find an old model message, and remove them
    for (iter = this->poseMsgs.begin(); iter != this->poseMsgs.end(); ++iter)
    {
      if ((*iter)->name() == _msg->name())
      {
        this->poseMsgs.erase(iter);
        break;
      }
    }
    this->poseMsgs.push_back(_msg);
  }
  this->receiveMutex->unlock();
}
  */

  /*
/////////////////////////////////////////////////
void LightListWidget::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    this->RemoveEntity(_msg->data());
  }
}
  */

