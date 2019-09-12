/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_MODELLISTWIDGETPRIVATE_HH_
#define _GAZEBO_GUI_MODELLISTWIDGETPRIVATE_HH_

#include <string>
#include <list>
#include <vector>
#include <deque>
#include <sdf/sdf.hh>
#include <ignition/msgs/plugin.pb.h>
#include <ignition/transport/Node.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

class QtTreePropertyBrowser;
class QtVariantEditorFactory;
class QtVariantPropertyManager;

namespace gazebo
{
  namespace gui
  {
    class ModelListWidgetPrivate
    {
      public: QTreeWidget *modelTreeWidget;
      public: QtTreePropertyBrowser *propTreeBrowser;

      public: transport::NodePtr node;
      public: transport::PublisherPtr requestPub;
      public: transport::PublisherPtr modelPub;
      public: transport::PublisherPtr scenePub;
      public: transport::PublisherPtr physicsPub;
      public: transport::PublisherPtr windPub;

      /// \brief Publisher for atmosphere messages.
      public: transport::PublisherPtr atmospherePub;

      public: transport::PublisherPtr lightPub;

      public: transport::SubscriberPtr responseSub;
      public: transport::SubscriberPtr requestSub;

      /// \brief GUI tree item.
      public: QTreeWidgetItem *guiItem;

      /// \brief Scene tree item.
      public: QTreeWidgetItem *sceneItem;

      /// \brief Physics tree item.
      public: QTreeWidgetItem *physicsItem;

      /// \brief Wind tree item.
      public: QTreeWidgetItem *windItem;

      /// \brief Atmosphere tree item.
      public: QTreeWidgetItem *atmosphereItem;

      /// \brief Models tree item.
      public: QTreeWidgetItem *modelsItem;

      /// \brief Lights tree item.
      public: QTreeWidgetItem *lightsItem;

      /// \brief Spherical coordinates tree item.
      public: QTreeWidgetItem *sphericalCoordItem;

      public: QtVariantPropertyManager *variantManager;
      public: QtVariantEditorFactory *variantFactory;
      public: std::mutex *propMutex, *receiveMutex;
      public: sdf::ElementPtr sdfElement;
      public: std::string selectedEntityName;
      public: bool fillingPropertyTree;
      public: QtProperty *selectedProperty;

      public: gazebo::msgs::Request *requestMsg;

      public: std::vector<event::ConnectionPtr> connections;

      typedef std::list<msgs::Model> ModelMsgs_L;
      public: ModelMsgs_L modelMsgs;

      typedef std::list<msgs::Light> LightMsgs_L;
      public: LightMsgs_L lightMsgs;

      typedef std::list<std::string> RemoveEntity_L;
      public: RemoveEntity_L removeEntityList;

      public: gazebo::msgs::Model modelMsg;
      public: gazebo::msgs::Link linkMsg;
      public: gazebo::msgs::Scene sceneMsg;
      public: gazebo::msgs::Joint jointMsg;

      /// \brief Keep latest plugin message.
      public: ignition::msgs::Plugin pluginMsg;

      public: gazebo::msgs::Physics physicsMsg;
      public: gazebo::msgs::Wind windMsg;

      /// \brief Keep latest atmosphere message.
      public: gazebo::msgs::Atmosphere atmosphereMsg;

      public: gazebo::msgs::Light lightMsg;
      public: gazebo::msgs::SphericalCoordinates sphericalCoordMsg;

      public: bool fillPropertyTree;
      public: std::deque<std::string> fillTypes;

      public: gazebo::msgs::Light::LightType lightType;

      /// \brief Type of physics engine.
      public: gazebo::msgs::Physics_Type physicsType;

      /// \brief Type of atmosphere model.
      public: gazebo::msgs::Atmosphere_Type atmosphereType;

      /// \brief Node for ignition transport communication.
      public: ignition::transport::Node ignNode;
    };
  }
}
#endif
