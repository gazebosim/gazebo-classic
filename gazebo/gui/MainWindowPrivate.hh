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
#ifndef GAZEBO_GUI_MAINWINDOWPRIVATE_HH_
#define GAZEBO_GUI_MAINWINDOWPRIVATE_HH_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gazebo/gazebo_config.h"
#include "gazebo/gui/qt.h"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

#ifdef HAVE_OCULUS
#include "gazebo/gui/OculusWindow.hh"
#endif

namespace gazebo
{
  namespace gui
  {
    class DataLogger;
    class Editor;
    class InsertModelWidget;
    class ModelListWidget;
    class RenderWidget;
    class SpaceNav;
    class ToolsWidget;
    class UserCmdHistory;

    class MainWindowPrivate
    {
      /// \brief Pointer to the render widget.
      public: RenderWidget *renderWidget = nullptr;

      /// \brief Pointer to the tools widget.
      public: ToolsWidget *toolsWidget = nullptr;

      /// \brief Pointer to the model list widget.
      public: ModelListWidget *modelListWidget = nullptr;

      /// \brief Transport node used for communication.
      public: transport::NodePtr node;

      /// \brief Publish world control messages.
      public: transport::PublisherPtr worldControlPub;

      /// \brief Publish server control messages.
      public: transport::PublisherPtr serverControlPub;

      /// \brief Publish request messages.
      public: transport::PublisherPtr requestPub;

      /// \brief Publish scene messages.
      public: transport::PublisherPtr scenePub;

      /// \brief Publish user command messages for the server to place in the
      /// undo queue.
      public: transport::PublisherPtr userCmdPub;

      /// \brief Subscribe to response messages.
      public: transport::SubscriberPtr responseSub;

      /// \brief Subscribe to response messages.
      public: transport::SubscriberPtr guiSub;

      /// \brief Subscribe to model info messages.
      public: transport::SubscriberPtr newEntitySub;

      /// \brief Subscribe to world modify messages.
      public: transport::SubscriberPtr worldModSub;

      /// \brief Subscriber to the light modify topic.
      public: transport::SubscriberPtr lightModifySub;

      /// \brief Subscriber to the light factory topic.
      public: transport::SubscriberPtr lightFactorySub;

      /// \brief Vector of event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief A map that associates physics_id's with entity names
      public: std::map<std::string, unsigned int> entities;

      /// \brief Message used to field requests.
      public: msgs::Request *requestMsg = nullptr;

      /// \brief The left-hand tab widget
      public: QTabWidget *tabWidget = nullptr;

      /// \brief Mainwindow's menubar
      public: QMenuBar *menuBar = nullptr;

      /// \brief The Edit menu.
      public: QMenu *editMenu = nullptr;

      /// \brief A layout for the menu bar.
      public: QHBoxLayout *menuLayout = nullptr;

      /// \brief Used to control size of each pane.
      public: QStackedWidget *leftColumn = nullptr;

      /// \brief Map of names to widgets in the leftColumn QStackedWidget
      public: std::map<std::string, int> leftColumnStack;

      /// \brief The filename set via "Save As". This filename is used by
      /// the "Save" feature.
      public: std::string saveFilename;

      /// \brief User specified step size for manually stepping the world
      public: int inputStepSize = 1;

      /// \brief Map of all the editors to their names.
      public: std::map<std::string, std::unique_ptr<Editor> > editors;

      /// \brief List of all the align action groups.
      public: std::vector<QActionGroup *> alignActionGroups;

      /// \brief Space navigator interface.
      public: SpaceNav *spacenav = nullptr;

#ifdef HAVE_OCULUS
      /// \brief Window for Oculus VR set.
      public: OculusWindow *oculusWindow = nullptr;
#endif

      /// \brief Buffer of plugin messages to process.
      public: std::vector<std::shared_ptr<msgs::Plugin const> > pluginMsgs;

      /// \brief Mutex to protect plugin loading.
      public: std::mutex pluginLoadMutex;

      /// \brief Splitter for the main window.
      public: QSplitter *splitter = nullptr;

      /// \brief Data logger dialog.
      public: DataLogger *dataLogger = nullptr;

      /// \brief Tab to insert models.
      public: InsertModelWidget *insertModel = nullptr;

      /// \brief Class which manages user commands and undoing / redoing them.
      public: UserCmdHistory *userCmdHistory = nullptr;

      // whether the file is atleast saved once or not.
      public: bool isSavedOnce = false;

      /// \brief Label for displaying the Gazebo classic EOL notice.
      public: QLabel *eolNotice = nullptr;
    };
  }
}
#endif
