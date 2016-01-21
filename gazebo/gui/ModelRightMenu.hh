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
#ifndef _MODELRIGHTMENU_HH_
#define _MODELRIGHTMENU_HH_

#include <map>
#include <vector>
#include <string>

#include "gazebo/common/KeyEvent.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/gui/ApplyWrenchDialog.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class ViewState;

    /// \class ModelRightMenu ModelRightMenu.hh gui/gui.hh
    /// \brief Displays a menu when the right mouse button has been pressed.
    class GZ_GUI_VISIBLE ModelRightMenu : public QObject
    {
      Q_OBJECT

      /// \enum EntityTypes
      /// \brief Unique identifiers for the type of entity this menu is
      /// attached to.
      public: enum EntityTypes {
                  /// \brief Model
                  MODEL,
                  /// \brief Light
                  LIGHT,
                  /// \brief Link
                  LINK
                };

      /// \brief Constructor
      public: ModelRightMenu();

      /// \brief Destructor
      public: virtual ~ModelRightMenu();

      /// \brief Initialize the right menu.
      /// \return True on success.
      public: bool Init();

      /// \brief Show the right menu.
      /// \param[in] _entityName Name of the entity that is active.
      /// \param[in] _pt Point on the GUI that has received the right-click
      /// request.
      /// \param[in] _type Type of the entity clicked.
      public: void Run(const std::string &_entityName, const QPoint &_pt,
          EntityTypes _type = MODEL);

      /// \brief QT callback when move to has been selected.
      private slots: void OnMoveTo();

      /// \brief QT callback when follow has been selected.
      private slots: void OnFollow();

      /// \brief QT callback when edit has been selected.
      private slots: void OnEdit();

      /// \brief QT callback when apply force/torque has been selected.
      private slots: void OnApplyWrench();

      /// \brief QT callback when delete has been selected.
      /// \param[in] _name Name of the entity to delete.
      private slots: void OnDelete(const std::string &_name="");

      /// \brief QT callback when snap below has been selected.
      // private slots: void OnSnapBelow();

      // private slots: void OnSkeleton();

      /// \brief Key release callback.
      /// \param[in] _event The key event.
      /// \return True if the key press was handled.
      private: bool OnKeyRelease(const common::KeyEvent &_event);

      /// \brief Request callback.
      /// \param[in] _msg Request message to process.
      private: void OnRequest(ConstRequestPtr &_msg);

      /// \brief Callback when window mode has changed.
      /// \param[in] _mode Window mode, such as "Simulation", "LogPlayback"...
      private: void OnWindowMode(const std::string &_mode);

      /// \brief Node for communication.
      private: transport::NodePtr node;

      /// \brief Subscriber to request messages.
      private: transport::SubscriberPtr requestSub;

      /// \brief Name of the active entity.
      private: std::string entityName;

      /// \brief Action for moving the camera to an object.
      private: QAction *moveToAct;

      /// \brief Action for attaching the camera to an entity.
      private: QAction *followAct;

      /// \brief Action for opening entity on Model Editor.
      private: QAction *editAct;

      /// \brief Action for applying force and torque to a model.
      private: QAction *applyWrenchAct;

      /// \brief All event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief The current window mode, such as "Simulation" or "LogPlayback".
      private: std::string windowMode;

      /// \brief Action for snapping an object to another object below the
      /// first.
      // private: QAction *snapBelowAct;
      // private: QAction *skeletonAct;

      /// \brief The various view states
      private: std::vector<ViewState*> viewStates;

      // The view state class is a friend for convenience
      private: friend class ViewState;
    };

    /// \class ViewState ViewState.hh gui/gui.hh
    /// \brief A class for managing view visualization states.
    /// Used by ModelRightMenu.
    class GZ_GUI_VISIBLE ViewState : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to the MOdelRightMenu
      /// \param[in] _checkRequest Name of the request to send when checked.
      /// \param[in] _uncheckRequest Name of the request to send when unchecked.
      public: ViewState(ModelRightMenu *_parent,
                  const std::string &_checkRequest,
                  const std::string &_uncheckRequest);

      /// \brief State of all the models for this view.
      public: std::map<std::string, bool> modelStates;

      /// \brief Action for this view.
      public: QAction *action;

      /// \brief True if the view visualization is enabled globally.
      public: bool globalEnable;

      /// \brief Pointer to the ModelRightMenu.
      public: ModelRightMenu *parent;

      /// \brief Name of the request to send when checked.
      public: std::string checkRequest;

      /// \brief Name of the request to send when unchecked.
      public: std::string uncheckRequest;

      /// \brief QT callback for the QAction.
      public slots: void Callback();
    };
  }
}
#endif
