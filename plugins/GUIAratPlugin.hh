/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _GUI_ARAT_PLUGIN_HH_
#define _GUI_ARAT_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/msgs/msgs.hh>
#include "gazebo/common/Events.hh"

#include <queue>
//#include <yaml-cpp/yaml.h>

namespace gazebo
{

    class ContactsWrapper
    {
      public: ConstContactsPtr msg;
      public: std::string name;
      public: ContactsWrapper(ConstContactsPtr m, std::string n) : msg(m), name(n){}
    };

    class GUIAratPlugin : public gazebo::GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: GUIAratPlugin();

      /// \brief Destructor
      public: virtual ~GUIAratPlugin();

      /// \brief Callback trigged when the button is pressed.
      protected slots: void OnButton();

      /// \brief Node used to establish communication with gzserver.
      private: gazebo::transport::NodePtr node;

      /// \brief Publisher of factory messages.
      private: gazebo::transport::PublisherPtr taskPub;

      /// \brief Subscriber to finger contact sensors.
      private: gazebo::transport::SubscriberPtr contactSub;

      /// \brief Number of the current task.
      private: int taskNum;

      /// \brief Maximum number tasks.
      /// \sa taskNum
      private: int maxTaskCount;

      private: QGraphicsScene *handScene;

      //TODO: config file
      private: const int handImgX = 250;
      private: const int handImgY = 250;

      private: std::string handImgFilename; // = "/home/jackie/gazebo_ws/src/gazebo/plugins/handsim.png";
      private: std::string fingerPtsFilename; // = "/home/jackie/gazebo_ws/src/gazebo/plugins/fingerpts.csv";

      private: const std::string fingerNames[5] = {"Th", "Ind", "Mid", "Ring", "Little"};

      //TODO: tune these values
      private: const int circleSize = 5;
      private: const unsigned char colorMin[3] = {255, 255, 0};
      private: const unsigned char colorMax[3] = {255, 0, 0};
      private: const float forceMin = 0;
      private: const float forceMax = 50;

      private: std::string handSide;
      
      private: std::vector<transport::SubscriberPtr> contactSubscribers;

      private: std::map<std::string, std::pair<int, int> > finger_points;

      private: std::map<std::string, QGraphicsEllipseItem*> contactGraphicsItems;

      private: void OnFingerContact(ConstContactsPtr &msg, std::string);

      private: void OnThumbContact(ConstContactsPtr &msg);

      private: void OnIndexContact(ConstContactsPtr &msg);

      private: void OnMiddleContact(ConstContactsPtr &msg);

      private: void OnRingContact(ConstContactsPtr &msg);

      private: void OnLittleContact(ConstContactsPtr &msg);

      private: std::string getTopicName(std::string fingerName);

      private: boost::mutex contactLock;

      private: std::vector<event::ConnectionPtr> connections;

      private: std::queue<ContactsWrapper> msgQueue;

      private: void PreRender();

    };
}
#endif
