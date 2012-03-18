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
#ifndef MAINWINDOW_HH
#define MAINWINDOW_HH

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
    class TimePanel;
    class GLWidget;

    class WorldPropertiesWidget;

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

      private slots: void New();
      private slots: void Open();
      private slots: void Import();
      private slots: void Save();
      private slots: void SaveAs();
      private slots: void About();
      private slots: void Play();
      private slots: void Pause();
      private slots: void Step();

      private slots: void NewModel();
      private slots: void EditWorldProperties();
      private slots: void Arrow();
      private slots: void RingPose();

      private slots: void CreateBox();
      private slots: void CreateSphere();
      private slots: void CreateCylinder();
      private slots: void CreateMesh();
      private slots: void CreatePointLight();
      private slots: void CreateSpotLight();
      private slots: void CreateDirectionalLight();
      private slots: void InsertModel();
      private slots: void ViewReset();
      private slots: void ViewFullScreen();
      private slots: void ViewFPS();
      private slots: void ViewOrbit();
      private slots: void OnResetWorld();

      private: void OnFullScreen(bool _value);
      private: void OnMoveMode(bool _mode);

      private: void CreateActions();
      private: void CreateMenus();
      private: void CreateToolbars();

      private: void OnModel(ConstModelPtr &_msg);
      private: void OnResponse(ConstResponsePtr &_msg);
      private: void OnWorldModify(ConstWorldModifyPtr &_msg);
      private: void OnManipMode(const std::string &_mode);

      private: QMenu *fileMenu;
      private: QMenu *editMenu;
      private: QMenu *viewMenu;
      private: QMenu *helpMenu;
      private: QToolBar *playToolbar;
      private: QToolBar *mouseToolbar;
      private: QToolBar *editToolbar;

      private: QAction *newAct;
      private: QAction *openAct;
      private: QAction *importAct;
      private: QAction *saveAct;
      private: QAction *saveAsAct;
      private: QAction *aboutAct;
      private: QAction *quitAct;

      private: QAction *newModelAct;
      private: QAction *resetWorldAct;
      private: QAction *editWorldPropertiesAct;

      private: QAction *playAct;
      private: QAction *pauseAct;
      private: QAction *stepAct;

      private: QAction *arrowAct;
      private: QAction *ringPoseAct;

      private: QAction *boxCreateAct;
      private: QAction *sphereCreateAct;
      private: QAction *cylinderCreateAct;
      private: QAction *meshCreateAct;
      private: QAction *pointLghtCreateAct;
      private: QAction *spotLghtCreateAct;
      private: QAction *dirLghtCreateAct;

      private: QAction *viewResetAct;
      private: QAction *viewFullScreenAct;
      private: QAction *viewFPSAct;
      private: QAction *viewOrbitAct;

      private: TimePanel *timePanel;
      private: RenderWidget *renderWidget;

      private: transport::NodePtr node;
      private: transport::PublisherPtr worldControlPub;
      private: transport::PublisherPtr serverControlPub;
      private: transport::PublisherPtr selectionPub;
      private: transport::PublisherPtr requestPub;
      private: transport::SubscriberPtr responseSub;
      private: transport::SubscriberPtr guiSub;
      private: transport::SubscriberPtr newEntitySub;
      private: transport::SubscriberPtr worldModSub;

      private: WorldPropertiesWidget *worldPropertiesWidget;
      private: QDockWidget *modelsDock;
      private: QDockWidget *insertModelsDock;

      private: std::vector<event::ConnectionPtr> connections;

      // A map that associates physics_id's with entity names
      private: std::map<std::string, unsigned int> entities;

      private: msgs::Request *requestMsg;
    };
  }
}

#endif
