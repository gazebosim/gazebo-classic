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

      private slots: void EditorSave();
      private slots: void EditorDiscard();
      private slots: void EditorDone();
      private slots: void EditorExit();

      /// \brief Callback when topic selection action.
      private slots: void SelectTopic();

      private: void OnFullScreen(bool _value);
      private: void OnMoveMode(bool _mode);

      private: void CreateActions();
      private: void CreateMenus();
      private: void CreateToolbars();

      private: void OnModel(ConstModelPtr &_msg);
      private: void OnResponse(ConstResponsePtr &_msg);
      private: void OnWorldModify(ConstWorldModifyPtr &_msg);
      private: void OnManipMode(const std::string &_mode);
      private: void OnSetSelectedEntity(const std::string &_name,
                                        const std::string &_mode);
      private: void OnStats(ConstWorldStatisticsPtr &_msg);
      private: void OnFinishModel();

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
      private: transport::SubscriberPtr responseSub;
      private: transport::SubscriberPtr guiSub;
      private: transport::SubscriberPtr newEntitySub, statsSub;
      private: transport::SubscriberPtr worldModSub;

      private: QDockWidget *toolsDock;

      private: std::vector<event::ConnectionPtr> connections;

      // A map that associates physics_id's with entity names
      private: std::map<std::string, unsigned int> entities;

      private: msgs::Request *requestMsg;

      // private: QTreeWidget *treeWidget;
//      priavte: QWidget
      private: BuildingEditorPalette *buildingEditorPalette;
      private: QTabWidget *tabWidget;
      private: QTabWidget *buildingEditorTabWidget;
      private: QMenuBar *menuBar;
      private: QMenuBar *editorMenuBar;
      private: QHBoxLayout *menuLayout;

      /// \brief The filename set via "Save As". This filename is used by
      /// the "Save" feature.
      private: std::string saveFilename;
    };

    class TreeViewDelegate: public QItemDelegate
    {
      Q_OBJECT
      public: TreeViewDelegate(QTreeView *_view, QWidget *_parent);

      public: void paint(QPainter *painter, const QStyleOptionViewItem &option,
                         const QModelIndex &index) const;

      public: virtual QSize sizeHint(const QStyleOptionViewItem &_opt,
                                     const QModelIndex &_index) const;
      private: QTreeView *view;
    };
  }
}

#endif
