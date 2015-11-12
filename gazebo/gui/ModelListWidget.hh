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
#ifndef _MODEL_LIST_WIDGET_HH_
#define _MODEL_LIST_WIDGET_HH_

#include <string>
#include <list>
#include <vector>
#include <deque>
#include <sdf/sdf.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

class QTreeWidget;
class QTreeWidgetItem;
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
    class ModelEditWidget;

    class GZ_GUI_VISIBLE ModelListWidget : public QWidget
    {
      Q_OBJECT
      public: ModelListWidget(QWidget *_parent = 0);
      public: virtual ~ModelListWidget();

      private slots: void OnModelSelection(QTreeWidgetItem *item, int column);
      private slots: void Update();
      private slots: void OnPropertyChanged(QtProperty *_item);
      private slots: void OnCustomContextMenu(const QPoint &_pt);
      private slots: void OnCurrentPropertyChanged(QtBrowserItem *_item);
      private: void OnSetSelectedEntity(const std::string &_name,
                                        const std::string &_mode);
      private: void OnResponse(ConstResponsePtr &_msg);

      private: void OnModelUpdate(const msgs::Model &_msg);

      /// \brief An event callback to handle light update msgs.
      /// \param[in] _msg Light message.
      private: void OnLightUpdate(const msgs::Light &_msg);

      private: void OnRequest(ConstRequestPtr &_msg);

      private: void OnRemoveScene(const std::string &_name);
      private: void OnCreateScene(const std::string &_name);

      private: void AddModelToList(const msgs::Model &_msg);

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

      private: void FillColorMsg(QtProperty *_item, msgs::Color *_msg);

      private: void FillVector3Msg(QtProperty *_item, msgs::Vector3d *_msg);

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

      private: QTreeWidgetItem *GetListItem(const std::string &_name,
                                            QTreeWidgetItem *_parent);

      private: void FillPropertyTree(const msgs::Model &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Link &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Collision &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Joint &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Surface &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Visual &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Geometry &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Scene &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Physics &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Atmosphere &_msg,
                                     QtProperty *_parent);

      private: void FillPropertyTree(const msgs::Light &_msg,
                                     QtProperty *_parent);

      private: void FillVector3dProperty(const msgs::Vector3d &_msg,
                                         QtProperty *_parent);

      private: void FillPoseProperty(const msgs::Pose &_msg,
                                     QtProperty *_parent);

      /// \brief Fill the property tree with spherical coordinates info.
      /// \param[in] _msg The spherical coordinates message.
      /// \param[in] _parent Pointer to the qtproperty which will receive
      /// the message data.
      private: void FillPropertyTree(const msgs::SphericalCoordinates &_msg,
                                     QtProperty *_parent);

      /// \brief Add a property to a parent property or to the property tree.
      /// \param[in] _item Pointer to the property to be added.
      /// \param[in] _parent Pointer to the parent property, if applicable.
      private: void AddProperty(QtProperty *_item, QtProperty *_parent);

      private: void ProcessModelMsgs();
      private: void ProcessLightMsgs();
      private: void ProcessRemoveEntity();

      public: void InitTransport(const std::string &_name ="");
      private: void ResetTree();
      private: void ResetScene();

      /// \brief Called when a model property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void ModelPropertyChanged(QtProperty *_item);

      /// \brief Called when a scene property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void ScenePropertyChanged(QtProperty *_item);

      private: void LightPropertyChanged(QtProperty *_item);

      /// \brief Called when a physics property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void PhysicsPropertyChanged(QtProperty *_item);

      /// \brief Called when an atmosphere property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void AtmospherePropertyChanged(QtProperty *_item);

      /// \brief Called when a GUI property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void GUIPropertyChanged(QtProperty *_item);

      private: QTreeWidget *modelTreeWidget;
      private: QtTreePropertyBrowser *propTreeBrowser;

      private: transport::NodePtr node;
      private: transport::PublisherPtr requestPub;
      private: transport::PublisherPtr modelPub;
      private: transport::PublisherPtr scenePub;
      private: transport::PublisherPtr physicsPub;
      private: transport::PublisherPtr atmospherePub;
      private: transport::PublisherPtr lightPub;

      private: transport::SubscriberPtr responseSub;
      private: transport::SubscriberPtr requestSub;

      /// \brief GUI tree item.
      private: QTreeWidgetItem *guiItem;

      /// \brief Scene tree item.
      private: QTreeWidgetItem *sceneItem;

      /// \brief Physics tree item.
      private: QTreeWidgetItem *physicsItem;

      /// \brief Atmosphere tree item.
      private: QTreeWidgetItem *atmosphereItem;

      /// \brief Models tree item.
      private: QTreeWidgetItem *modelsItem;

      /// \brief Lights tree item.
      private: QTreeWidgetItem *lightsItem;

      /// \brief Spherical coordinates tree item.
      private: QTreeWidgetItem *sphericalCoordItem;

      private: QtVariantPropertyManager *variantManager;
      private: QtVariantEditorFactory *variantFactory;
      private: boost::mutex *propMutex, *receiveMutex;
      private: sdf::ElementPtr sdfElement;
      private: std::string selectedEntityName;
      private: bool fillingPropertyTree;
      private: QtProperty *selectedProperty;

      private: msgs::Request *requestMsg;

      private: std::vector<event::ConnectionPtr> connections;

      typedef std::list<msgs::Model> ModelMsgs_L;
      private: ModelMsgs_L modelMsgs;

      typedef std::list<msgs::Light> LightMsgs_L;
      private: LightMsgs_L lightMsgs;

      typedef std::list<std::string> RemoveEntity_L;
      private: RemoveEntity_L removeEntityList;

      private: msgs::Model modelMsg;
      private: msgs::Link linkMsg;
      private: msgs::Scene sceneMsg;
      private: msgs::Joint jointMsg;
      private: msgs::Physics physicsMsg;
      private: msgs::Atmosphere atmosphereMsg;
      private: msgs::Light lightMsg;
      private: msgs::SphericalCoordinates sphericalCoordMsg;

      private: bool fillPropertyTree;
      private: std::deque<std::string> fillTypes;

      private: msgs::Light::LightType lightType;

      /// \brief Type of physics engine.
      private: msgs::Physics_Type physicsType;

      /// \brief Type of atmosphere model.
      private: msgs::Atmosphere_Type atmosphereType;
    };

    class GZ_GUI_VISIBLE ModelListSheetDelegate: public QItemDelegate
    {
      Q_OBJECT
      public: ModelListSheetDelegate(QTreeView *view, QWidget *parent);

      public: virtual void paint(QPainter *painter,
                  const QStyleOptionViewItem &option,
                  const QModelIndex &index) const;

      public: virtual QSize sizeHint(const QStyleOptionViewItem &opt,
                  const QModelIndex &index) const;

      private: QTreeView *m_view;
    };
  }
}
#endif
