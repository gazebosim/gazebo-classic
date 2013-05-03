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
#ifndef _MAINWINDOW_HH_
#define _MAINWINDOW_HH_

#include <map>
#include <string>
#include <vector>

#include "gui/qt.h"
#include "common/Event.hh"
#include "msgs/MessageTypes.hh"
#include "transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class RenderWidget;
    class ToolsWidget;
    class ModelListWidget;
    class BuildingEditorPalette;

    class MainWindow : public QMainWindow
    {
      Q_OBJECT

      public: MainWindow();
      public: virtual ~MainWindow();

      public: void Load();
      public: void Init();

      public: unsigned int GetEntityId(const std::string &_name);
      public: bool HasEntityName(const std::string &_name);

      protected: void closeEvent(QCloseEvent *_event);

      private: void OnGUI(ConstGUIPtr &_msg);


      private slots: void ItemSelected(QTreeWidgetItem *, int);
      private slots: void New();
      private slots: void Open();

      /// \brief Open a log file
      private slots: void OpenLog();

      private slots: void Import();
      private slots: void Save();
      private slots: void SaveAs();
      private slots: void About();
      private slots: void Play();
      private slots: void Pause();
      private slots: void Step();

      private slots: void NewModel();
      private slots: void Arrow();
      private slots: void Translate();
      private slots: void Rotate();

      private slots: void CreateBox();
      private slots: void CreateSphere();
      private slots: void CreateCylinder();
      private slots: void CreateMesh();
      private slots: void CreatePointLight();
      private slots: void CreateSpotLight();
      private slots: void CreateDirectionalLight();

      /// \brief Qt callback when the screenshot action is triggered
      private slots: void CaptureScreenshot();

      private slots: void InsertModel();
      private slots: void ShowGrid();
      private slots: void ShowCollisions();
      private slots: void ShowJoints();
      private slots: void ShowContacts();
      private slots: void ShowCOM();
      private slots: void Reset();
      private slots: void FullScreen();
      private slots: void FPS();
      private slots: void Orbit();
      private slots: void OnResetModelOnly();
      private slots: void OnResetWorld();
      private slots: void SetTransparent();
      private slots: void OnEditBuilding();
      private slots: void SetWireframe();

      /// \brief Qt call back when the play action state changes
      private slots: void OnPlayActionChanged();

      /// \brief Qt callback when the building editor's save action is
      /// triggered.
      private slots: void BuildingEditorSave();

      /// \brief Qt callback when the building editor's discard action is
      /// triggered.
      private slots: void BuildingEditorDiscard();

      /// \brief Qt callback when the building editor's done action is
      /// triggered.
      private slots: void BuildingEditorDone();

      /// \brief Qt callback when the building editor's exit action is
      /// triggered.
      private slots: void BuildingEditorExit();

      /// \brief QT slot to open the data logger utility
      private slots: void DataLogger();

      /// \brief Callback when topic selection action.
      private slots: void SelectTopic();

      /// \brief Callback for diagnostics action.
      private slots: void Diagnostics();

      /// \brief Receives status messages from the server.
      /// \param[in] _msg Message pointer.
      private: void OnServerStatus(ConstGzStringPtr &_msg);

      private: void OnFullScreen(bool _value);
      private: void OnMoveMode(bool _mode);

      private: void CreateActions();
      private: void CreateMenus();
      private: void CreateToolbars();

      /// \brief Attach Gazebo's main menu bar
      private: void AttachMainMenuBar();

      /// \brief Attach building editor's menu bar
      private: void AttachEditorMenuBar();

      private: void OnModel(ConstModelPtr &_msg);
      private: void OnResponse(ConstResponsePtr &_msg);
      private: void OnWorldModify(ConstWorldModifyPtr &_msg);
      private: void OnManipMode(const std::string &_mode);
      private: void OnSetSelectedEntity(const std::string &_name,
                                        const std::string &_mode);
      private: void OnStats(ConstWorldStatisticsPtr &_msg);

      /// \brief Callback from the building editor when the building model
      /// has been completed.
      private: void OnFinishBuilding();

      /// \brief Handle event for changing the manual step size.
      /// \param[in] _value New input step size.
      private: void OnInputStepSizeChanged(int _value);

      private: QToolBar *playToolbar;

      private: RenderWidget *renderWidget;
      private: ToolsWidget *toolsWidget;
      private: ModelListWidget *modelListWidget;

      private: transport::NodePtr node;
      private: transport::PublisherPtr worldControlPub;
      private: transport::PublisherPtr serverControlPub;
      private: transport::PublisherPtr selectionPub;
      private: transport::PublisherPtr requestPub;
      private: transport::PublisherPtr scenePub;

      /// \brief Receives status messages from the server
      private: transport::SubscriberPtr serverControlSub;
      private: transport::SubscriberPtr responseSub;
      private: transport::SubscriberPtr guiSub;
      private: transport::SubscriberPtr newEntitySub, statsSub;
      private: transport::SubscriberPtr worldModSub;

      private: QDockWidget *toolsDock;

      private: std::vector<event::ConnectionPtr> connections;

      // A map that associates physics_id's with entity names
      private: std::map<std::string, unsigned int> entities;

      private: msgs::Request *requestMsg;

      /// \brief Building editor palette that contains different drawing modes
      private: BuildingEditorPalette *buildingEditorPalette;

      /// \brief Tab widget that holds the building editor palette
      private: QTabWidget *buildingEditorTabWidget;

      private: QTabWidget *tabWidget;
      private: QMenuBar *menuBar;

      /// \brief A layout for the menu bar.
      private: QHBoxLayout *menuLayout;

      /// \brief The filename set via "Save As". This filename is used by
      /// the "Save" feature.
      private: std::string saveFilename;

      /// \brief User specified step size for manually stepping the world
      private: int inputStepSize;
    };
  }
}

#endif
