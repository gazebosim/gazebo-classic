/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _MODELRIGHTMENU_HH_
#define _MODELRIGHTMENU_HH_

#include <map>
#include <string>
#include "gui/qt.h"
#include "msgs/msgs.hh"
#include "transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class ModelRightMenu : public QObject
    {
      Q_OBJECT
      public: ModelRightMenu();
      public: virtual ~ModelRightMenu();

      public: void Run(const std::string &_modelName, const QPoint &_pt);

      private slots: void OnSnapBelow();
      private slots: void OnMoveTo();
      private slots: void OnDelete(const std::string &_name="");
      private slots: void OnFollow();
      private slots: void OnShowCollision();
      private slots: void OnShowJoints();
      private slots: void OnShowCOM();
      private slots: void OnTransparent();
      private slots: void OnSkeleton();

      private: std::string modelName;

      private: QAction *moveToAction;
      private: QAction *transparentAct;

      private: QAction *snapBelowAction;
      private: QAction *followAction;
      private: QAction *showCollisionAction;
      private: QAction *skeletonAction;
      private: QAction *showJointsAction;
      private: QAction *showCOMAction;

      private: transport::NodePtr node;
      private: transport::PublisherPtr requestPub;
      private: msgs::Request *requestMsg;

      private: std::map<std::string, bool> showCollisionsActionState;
      private: std::map<std::string, bool> showJointsActionState;
      private: std::map<std::string, bool> showCOMActionState;
      private: std::map<std::string, bool> transparentActionState;
      private: std::map<std::string, bool> skeletonActionState;
    };
  }
}
#endif
