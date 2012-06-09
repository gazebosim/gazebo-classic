/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef __LIGHTRIGHTMENU_HH__
#define __LIGHTRIGHTMENU_HH__

#include <map>
#include <string>
#include <gazebo/gui/qt.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/TransportTypes.hh>

namespace gazebo
{
  namespace gui
  {
    class LightRightMenu : public QObject
    {
      Q_OBJECT
      public: LightRightMenu();
      public: virtual ~LightRightMenu();

      public: void Run(const std::string &_modelName, const QPoint &_pt);

      private slots: void OnMoveTo();
      private slots: void OnDelete();

      private: std::string lightName;

      private: QAction *moveToAction;
      private: QAction *deleteAction;

      private: transport::NodePtr node;
      private: transport::PublisherPtr requestPub;
      private: msgs::Request *requestMsg;
    };
  }
}
#endif
