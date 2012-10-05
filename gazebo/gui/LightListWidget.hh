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
#ifndef _LIGHT_LIST_WIDGET_HH_
#define _LIGHT_LIST_WIDGET_HH_

#include <string>
#include <list>
#include <vector>

#include "gui/qt.h"
#include "sdf/sdf.hh"
#include "msgs/msgs.hh"
#include "transport/TransportTypes.hh"
#include "rendering/RenderTypes.hh"

class QListWidget;
class QListWidgetItem;
class QPushButton;
class QtTreePropertyBrowser;
class QtVariantPropertyManager;
class QtProperty;
class QtTreePropertyItem;
class QtBrowserItem;
class QtVariantEditorFactory;

namespace boost
{
  class recursive_mutex;
  class mutex;
}

namespace gazebo
{
  namespace gui
  {
    class LightListWidget : public QWidget
    {
      Q_OBJECT
      public: LightListWidget(QWidget *_parent = 0);
      public: virtual ~LightListWidget();
      private: QListWidgetItem *GetLightListItem(const std::string &_string);
      private: void InitTransport(const std::string &_name ="");
      private: void OnCreateScene(const std::string &_name);
      private: void OnRemoveScene(const std::string &_name);
      private: void ProcessLightMsgs();
      private: void OnLightMsg(ConstLightPtr &_msg);
      private: void OnSetSelectedEntity(const std::string &_name);

      private slots: void OnCustomContextMenu(const QPoint &_pt);
      private slots: void OnLightSelection(QListWidgetItem *item);
      private slots: void Update();

      /*
      private slots: void OnPropertyChanged(QtProperty *_item);
      private slots: void OnCurrentPropertyChanged(QtBrowserItem *_item);
      private: void OnResponse(ConstResponsePtr &_msg);

      private: void OnModelUpdate(const msgs::Model &_msg);

      private: void OnRequest(ConstRequestPtr &_msg);

      private: void OnPose(ConstPosePtr &_msg);


      private: void AddModelToList(const msgs::Model &_msg);

      private: void FillPropertyTree(const msgs::Model &_msg,
                                     QtProperty *_parentItem);

      private: void FillPropertyTree(const msgs::Link &_msg,
                                     QtProperty *_parentItem);

      private: void FillPropertyTree(const msgs::Collision &_msg,
                                     QtProperty *_parent);

      private: void FillMsgField(QtProperty *_item,
                   google::protobuf::Message *_message,
                   const google::protobuf::Reflection *_reflection,
                   const google::protobuf::FieldDescriptor *_field);

      private: void FillMsg(QtProperty *_item,
                   google::protobuf::Message *_message,
                   const google::protobuf::Descriptor *_descriptor,
                   QtProperty *_changedItem);

      private: void FillGeometryMsg(QtProperty *_item,
                   google::protobuf::Message *_message,
                   const google::protobuf::Descriptor *_descriptor,
                   QtProperty *_changedItem);

      private: void FillPoseMsg(QtProperty *_item,
                   google::protobuf::Message *_message,
                   const google::protobuf::Descriptor *_descriptor);

      private: QtProperty *PopChildItem(QList<QtProperty*> &_list,
                                        const std::string &_name);

      private: QtProperty *GetParentItemValue(const std::string &_name);
      private: QtProperty *GetParentItemValue(QtProperty *_item,
                                           const std::string &_name);

      private: QtProperty *GetParentItem(const std::string &_name);
      private: QtProperty *GetParentItem(QtProperty *_item,
                                           const std::string &_name);

      private: QtProperty *GetChildItemValue(const std::string &_name);
      private: QtProperty *GetChildItemValue(QtProperty *_item,
                                             const std::string &_name);

      private: QtProperty *GetChildItem(const std::string &_name);
      private: QtProperty *GetChildItem(QtProperty *_item,
                                        const std::string &_name);

      private: bool HasChildItem(QtProperty *_parent, QtProperty *_child);

      private: void RemoveEntity(const std::string &_name);


      private: void FillVector3dProperty(const msgs::Vector3d &_msg,
                                         QtProperty *_parent);

      private: void FillPoseProperty(const msgs::Pose &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Surface &_msg,
                                       QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Visual &_msg,
                                       QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Geometry &_msg,
                                       QtProperty *_parent);

      private: void ProcessPoseMsgs();
               */


      private: QListWidget *lightListWidget;
      private: QtTreePropertyBrowser *propTreeBrowser;

      private: transport::NodePtr node;
      private: transport::SubscriberPtr lightSub;

      private: transport::PublisherPtr requestPub, modelPub;
      private: transport::SubscriberPtr responseSub;
      private: transport::SubscriberPtr requestSub;
      private: transport::SubscriberPtr poseSub;

      private: QtVariantPropertyManager *variantManager;
      private: QtVariantEditorFactory *variantFactory;
      private: boost::mutex *propMutex, *receiveMutex;
      private: sdf::ElementPtr sdfElement;
      private: std::string selectedModelName;
      private: bool fillingPropertyTree;
      private: QtProperty *selectedProperty;

      private: msgs::Request *requestMsg;

      private: std::vector<event::ConnectionPtr> connections;

      typedef std::list<boost::shared_ptr<msgs::Pose const> > PoseMsgs_L;
      private: PoseMsgs_L poseMsgs;

      typedef std::list<boost::shared_ptr<msgs::Light const> > LightMsgs_L;
      private: LightMsgs_L lightMsgs;

      private: bool fillPropertyTree;
    };
  }
}
#endif
