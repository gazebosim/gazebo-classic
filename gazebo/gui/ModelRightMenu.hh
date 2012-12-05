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

    /// \class ModelRightMenu ModelRightMenu.hh gui/gui.hh
    /// \brief Displays a menu when the right mouse button has been pressed.
    class ModelRightMenu : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: ModelRightMenu();

      /// \brief Destructor
      public: virtual ~ModelRightMenu();

      /// \brief Show the right menu.
      /// \param[in] _modelName Name of the model that is active.
      /// \param[in] _pt Point on the GUI that has received the right-click
      /// request.
      public: void Run(const std::string &_modelName, const QPoint &_pt);

      /// \brief QT callback when move to has been selected.
      private slots: void OnMoveTo();

      /// \brief QT callback when show collisions has been selected.
      private slots: void OnShowCollision();

      /// \brief QT callback when show joints has been selected.
      private slots: void OnShowJoints();

      /// \brief QT callback when show COM has been selected.
      private slots: void OnShowCOM();

      /// \brief QT callback when transparent has been selected.
      private slots: void OnTransparent();

      /// \brief QT callback when delete has been selected.
      private slots: void OnDelete(const std::string &_name="");

      /// \brief QT callback when snap below has been selected.
      private slots: void OnSnapBelow();

      // private slots: void OnFollow();
      // private slots: void OnSkeleton();

      /// \brief Request callback.
      /// \param[in] _msg Request message to process.
      private: void OnRequest(ConstRequestPtr &_msg);

      /// \brief Helper function to set all values in a map to _value.
      /// \param[in] _map The map to set.
      /// \param[in] _value The value to set the map to.
      private: void SetMap(std::map<std::string, bool> &_map, bool _value);

      /// \brief Node for communication.
      private: transport::NodePtr node;

      /// \brief Subscriber to request messages.
      private: transport::SubscriberPtr requestSub;

      /// \brief Name of the active model.
      private: std::string modelName;

      /// \brief Action for moving the camera to an object.
      private: QAction *moveToAct;

      /// \brief Action for setting an object transparent.
      private: QAction *transparentAct;

      /// \brief Action for showing the collisions of an object.
      private: QAction *showCollisionAct;

      /// \brief Action for showing the joints of an object.
      private: QAction *showJointsAct;

      /// \brief Action for showing the COM of an object.
      private: QAction *showCOMAct;

      /// \brief Action for snapping an object to another object below the
      /// first.
      private: QAction *snapBelowAct;

      // private: QAction *followAct;
      // private: QAction *skeletonAct;

      /// \brief True when all collision objects should be shown.
      private: bool showAllCollisions;

      /// \brief Map of model names to collision visualization state
      private: std::map<std::string, bool> showCollisionsActionState;

      /// \brief True when all joints should be shown.
      private: bool showAllJoints;

      /// \brief Map of model names to joint visualization state
      private: std::map<std::string, bool> showJointsActionState;

      /// \brief True when all COM should be shown.
      private: bool showAllCOM;

      /// \brief Map of model names to COM visualization state
      private: std::map<std::string, bool> showCOMActionState;

      /// \brief True when all visuals are transparent.
      private: bool allTransparent;

      /// \brief Map of model names to transparent visualization state
      private: std::map<std::string, bool> transparentActionState;

      /// \brief Map of model names to skeleton visualization state
      // private: std::map<std::string, bool> skeletonActionState;
    };
  }
}
#endif
