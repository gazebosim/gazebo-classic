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
#ifndef GAZEBO_GUI_MODELLISTWIDGET_HH_
#define GAZEBO_GUI_MODELLISTWIDGET_HH_

#include <memory>
#include <string>
#include <QItemDelegate>
#include <QObject>
#include <QWidget>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

class QtBrowserItem;
class QtProperty;
class QTreeView;
class QTreeWidgetItem;

namespace gazebo
{
  namespace gui
  {
    class ModelListWidgetPrivate;

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

      private: QtProperty *ParentItemValue(const std::string &_name);
      private: QtProperty *ParentItemValue(QtProperty *_item,
                                           const std::string &_name);

      private: QtProperty *ParentItem(const std::string &_name);
      private: QtProperty *ParentItem(QtProperty *_item,
                                      const std::string &_name);

      private: QtProperty *ChildItemValue(const std::string &_name);
      private: QtProperty *ChildItemValue(QtProperty *_item,
                                          const std::string &_name);

      private: QtProperty *ChildItem(const std::string &_name);
      private: QtProperty *ChildItem(QtProperty *_item,
                                     const std::string &_name);

      private: bool HasChildItem(QtProperty *_parent, QtProperty *_child);

      private: void RemoveEntity(const std::string &_name);

      private: QTreeWidgetItem *ListItem(const std::string &_name,
                                         QTreeWidgetItem *_parent);

      private: void FillPropertyTree(const msgs::Model &_msg,
                                     QtProperty *_parent);

      /// \brief Fill the property tree with plugin info.
      /// \param[in] _msg The plugin message.
      /// \param[in] _parent Pointer to the qtproperty which will receive
      /// the message data.
      private: void FillPropertyTree(const msgs::Plugin &_msg,
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

      private: void FillPropertyTree(const msgs::Wind &_msg,
                                     QtProperty *_parent);

      /// \brief Fill the property tree with atmosphere info.
      /// \param[in] _msg The atmosphere message.
      /// \param[in] _parent Pointer to the qtproperty which will receive
      /// the message data.
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

      /// \brief Fill the property tree with user camera info taken from
      /// rendering.
      private: void FillUserCamera();

      /// \brief Fill the property tree with grid info taken from rendering.
      private: void FillGrid();

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

      /// \brief Called when a wind property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void WindPropertyChanged(QtProperty *_item);

      /// \brief Called when an atmosphere property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void AtmospherePropertyChanged(QtProperty *_item);

      /// \brief Called when a GUI property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void GUIPropertyChanged(QtProperty *_item);

      /// \brief Called when a GUI camera property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void GUICameraPropertyChanged(QtProperty *_item);

      /// \brief Called when a GUI grid property is changed by the user.
      /// \param[in] _item The item that was changed.
      private: void GUIGridPropertyChanged(QtProperty *_item);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ModelListWidgetPrivate> dataPtr;
    };
  }
}
#endif
