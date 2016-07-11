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
#ifndef _GAZEBO_GUI_MODELLISTWIDGETPRIVATE_HH_
#define _GAZEBO_GUI_MODELLISTWIDGETPRIVATE_HH_

#include <string>
#include <list>
#include <vector>
#include <deque>
#include <sdf/sdf.hh>

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

      public: msgs::Request *requestMsg;

      public: std::vector<event::ConnectionPtr> connections;

      typedef std::list<msgs::Model> ModelMsgs_L;
      public: ModelMsgs_L modelMsgs;

      typedef std::list<msgs::Light> LightMsgs_L;
      public: LightMsgs_L lightMsgs;

      typedef std::list<std::string> RemoveEntity_L;
      public: RemoveEntity_L removeEntityList;

      public: msgs::Model modelMsg;
      public: msgs::Link linkMsg;
      public: msgs::Scene sceneMsg;
      public: msgs::Joint jointMsg;

      /// \brief Keep latest plugin message.
      public: msgs::Plugin pluginMsg;

      public: msgs::Physics physicsMsg;
      public: msgs::Wind windMsg;

      /// \brief Keep latest atmosphere message.
      public: msgs::Atmosphere atmosphereMsg;

      public: msgs::Light lightMsg;
      public: msgs::SphericalCoordinates sphericalCoordMsg;

      public: bool fillPropertyTree;
      public: std::deque<std::string> fillTypes;

      public: msgs::Light::LightType lightType;

      /// \brief Type of physics engine.
      public: msgs::Physics_Type physicsType;

      /// \brief Type of atmosphere model.
      public: msgs::Atmosphere_Type atmosphereType;
    };
  }
}
#endif
