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
#include "gazebo/math/gzmath.hh"

#include <queue>

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
      //protected slots: void OnButton();
      protected slots: void OnTaskSent(std::string id, QTextDocument* instructions, int index);
      protected slots: void OnResetClicked(); 
      protected slots: void OnNextClicked(); 
      private: void PublishTaskMessage(std::string task_name); 

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

      private: const std::string fingerNames[5] = {"Th", "Ind", "Mid", "Ring", "Little"};

      private: int handImgX;
      private: int handImgY;

      private: std::string handImgFilename;
      private: std::string configFilename;

      //TODO: tune these values
      private: int circleSize;
      private: math::Vector2d iconSize;
      private: math::Vector3 colorMin;
      private: math::Vector3 colorMax;
      private: float forceMin;
      private: float forceMax;

      private: std::string handSide;
      
      private: std::vector<transport::SubscriberPtr> contactSubscribers;

      private: std::map<std::string, math::Vector2d > finger_points;

      private: std::map<std::string, QGraphicsEllipseItem*> contactGraphicsItems;

      private: QTextEdit* instructionsView;

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
  
      private: std::vector<std::string> taskList;
      private: std::vector<QTextDocument*> instructionsList;
      private: int currentTaskIndex;

      private: void PreRender();

    };

    class QTaskButton : public QPushButton
    {
      Q_OBJECT
      public: QTaskButton();

      public: void SetTaskId(std::string task_id);
      public: void SetTaskInstructionsDocument(QTextDocument* instr);
      public: void SetIndex(int i);

      public slots: void OnButton();

      private: std::string id;
      private: QTextDocument* instructions;
      private: int index; //The canonical index at which this task is stored

      signals: void SendTask(std::string id, QTextDocument* instructions, int index);
    };
}
#endif
