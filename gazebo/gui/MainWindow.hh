/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_MAINWINDOW_HH_
#define GAZEBO_GUI_MAINWINDOW_HH_

#include <memory>
#include <string>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

#ifdef HAVE_OCULUS
#include "gazebo/gui/OculusWindow.hh"
#endif

namespace gazebo
{
  namespace gui
  {
    class Editor;
    class RenderWidget;

    // Forward declare private data
    class MainWindowPrivate;

    class GZ_GUI_VISIBLE MainWindow : public QMainWindow
    {
      Q_OBJECT

      /// \brief Constructor
      public: MainWindow();

      /// \brief Destructor
      public: virtual ~MainWindow();

      /// \brief Load the mainwindow
      public: void Load();

      /// \brief Initialization
      public: void Init();

      /// \brief Get an entity id
      /// \param[in] _name The name of the entity
      /// \return The entity id
      public: unsigned int EntityId(const std::string &_name);

      /// \brief Has an entity name
      /// \param[in] _name The entity name
      /// \return True if the entity has a name
      public: bool HasEntityName(const std::string &_name);

      /// \brief Add a widget to the left column stack of widgets.
      /// \param[in] _name Name of the widget
      /// \param[in] _widget Pointer to the widget to add.
      public: void AddToLeftColumn(const std::string &_name, QWidget *_widget);

      /// \brief Show a widget in the left column.
      /// \sa AddToLeftColumn
      /// \param[in] _name Name of the widge to show. The widget must have
      /// been added using AddToLeftColumn. The string "default" will show
      /// the main tab.
      public: void ShowLeftColumnWidget(const std::string &_name = "default");

      /// \brief Get a pointer to the render widget.
      /// \return A pointer to the render widget.
      public: gui::RenderWidget *RenderWidget() const;

      /// \brief Returns the state of the simulation, true if paused.
      /// \return True if paused, false otherwise.
      public: bool IsPaused() const;

      /// \brief Play simulation.
      public slots: void Play();

      /// \brief Pause simulation.
      public slots: void Pause();

      /// \brief Set whether the left pane is visible
      /// \param[in] _on True to show the left pane, false to hide.
      public: void SetLeftPaneVisibility(bool _on);

      /// \brief Add a menu to the main window menu bar.
      /// \param[in] _menu Menu to be added.
      public: void AddMenu(QMenu *_menu);

      /// \brief Show a custom menubar. If NULL is used, the default menubar
      /// is shown.
      /// \param[in] _bar The menubar to show. NULL will show the default
      /// menubar.
      public: void ShowMenuBar(QMenuBar *_bar = NULL);

      /// \brief Create a new action with information from the given action,
      /// such as text and tooltip. The new action triggers the original action
      /// and follows its checked state. This is used for example to have the
      /// "same" action on the main window menu and the model editor menu,
      /// since an action can't be added to 2 different menus.
      /// \param[in] _action Action to be cloned.
      /// \param[in] _parent Parent for the new action.
      /// \return The new action.
      public: QAction *CloneAction(QAction *_action, QObject *_parent);

      /// \brief Get an editor by name
      /// \param[in] _name Name of the editor.
      /// \return Pointer to the editor.
      public: gui::Editor *Editor(const std::string &_name) const;

      /// \brief A signal to trigger loading of GUI plugins.
      signals: void AddPlugins();

      /// \brief A signal to track a visual.
      /// \param[in] _visualName Name of the visual to attach the camera to.
      signals: void TrackVisual(const std::string &_visualName);

      /// \brief A signal to indicate the main window is about to close.
      signals: void Close();

      protected: void closeEvent(QCloseEvent *_event);

      private: void OnGUI(ConstGUIPtr &_msg);

      private slots: void ItemSelected(QTreeWidgetItem *, int);
      private slots: void New();
      private slots: void Open();
      private slots: void Save();
      private slots: void SaveAs();

      /// \brief Save GUI configuration to INI file.
      private slots: void SaveINI();

      /// \brief Clone a simulation.
      private slots: void Clone();

      /// \brief Qt callback when the hotkey chart is triggered,
      private slots: void HotkeyChart();

      /// \brief Qt callback when about is triggered,
      private slots: void About();

      private slots: void Step();

      /// \brief Qt callback when the arrow mode is triggered.
      private slots: void Arrow();

      /// \brief Qt callback when the translate mode is triggered.
      private slots: void Translate();

      /// \brief Qt callback when the rotate mode is triggered.
      private slots: void Rotate();

      /// \brief Qt callback when the scale mode is triggered.
      private slots: void Scale();

      /// \brief Qt callback when the main align action is triggered. Currently
      /// just resets the child align actions.
      private slots: void Align();

      /// \brief Qt callback when the snap mode is triggered.
      private slots: void Snap();

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

      /// \brief Qt callback when the show grid action is triggered.
      private slots: void ShowGrid();

      /// \brief Qt callback when the show origin action is triggered.
      private slots: void ShowOrigin();

      /// \brief Qt callback when the show collisions action is triggered.
      private slots: void ShowCollisions();

      /// \brief Qt callback when the show joints action is triggered.
      private slots: void ShowJoints();

      /// \brief Qt callback when the show contacts action is triggered.
      private slots: void ShowContacts();

      /// \brief Qt callback when the show center of mass action is triggered.
      private slots: void ShowCOM();

      /// \brief Qt callback when the show inertia action is triggered.
      private slots: void ShowInertia();

      /// \brief Qt callback when the show link frame action is triggered.
      private slots: void ShowLinkFrame();

      /// \brief Qt callback when the show skeleton action is triggered.
      private slots: void ShowSkeleton();

      /// \brief Qt callback when the full screen action is triggered.
      private slots: void FullScreen();

      /// \brief Qt callback when the show toolbars action is triggered.
      private slots: void ShowToolbars();

      private slots: void FPS();
      private slots: void Orbit();
      private slots: void ViewOculus();
      private slots: void OnResetModelOnly();
      private slots: void OnResetWorld();
      private slots: void SetTransparent();
      private slots: void SetWireframe();

      /// \brief Qt callback when the show GUI overlays action is triggered.
      private slots: void ShowGUIOverlays();

      /// \brief QT slot to open the data logger utility
      private slots: void DataLogger();

      /// \brief QT callback when the data logger is shut down.
      private slots: void OnDataLoggerClosed();

      /// \brief Callback when topic selection action.
      private slots: void SelectTopic();

      /// \brief Callback for plot action.
      private slots: void Plot();

      /// \brief Callback for adding plugins.
      private slots: void OnAddPlugins();

      /// \brief Callback for tracking a visual.
      /// \param[in] _visualName Name of the visual to attach the camera to.
      private slots: void OnTrackVisual(const std::string &_visualName);

      /// \brief Qt call back when one of the editor actions is triggered.
      /// \param[in] _action Action in the group which was triggered.
      private slots: void OnEditorGroup(QAction *_action);

      /// \brief Toggle full screen display.
      /// \param[in] _value True to display in full screen mode.
      private: void OnFullScreen(bool _value);

      /// \brief Toggle toolbars display.
      /// \param[in] _value True to display toolbars.
      private: void OnShowToolbars(bool _value);

      private: void OnMoveMode(bool _mode);

      /// \brief Create most of the actions.
      private: void CreateActions();

      /// \brief Delete the actions created in CreateActions.
      private: void DeleteActions();

      /// \brief Create menus.
      private: void CreateMenus();

      /// \brief Create the main menu bar.
      private: void CreateMenuBar();

      /// \brief Create all the editors.
      private: void CreateEditors();

      private: void OnModel(ConstModelPtr &_msg);

      /// \brief Light message callback.
      /// \param[in] _msg Pointer to the light message.
      private: void OnLight(ConstLightPtr &_msg);

      private: void OnResponse(ConstResponsePtr &_msg);
      private: void OnWorldModify(ConstWorldModifyPtr &_msg);
      private: void OnManipMode(const std::string &_mode);
      private: void OnSetSelectedEntity(const std::string &_name,
                                        const std::string &_mode);

      /// \brief Handle event for changing the manual step size.
      /// \param[in] _value New input step size.
      private: void OnInputStepSizeChanged(int _value);

      /// \brief Handle follow model user event.
      /// \param[in] _modelName Name of the model that is being followed.
      private: void OnFollow(const std::string &_modelName);

      /// \brief Helper function that creates a greyed out icon for an
      /// action. This lets the action have a visible disabled state.
      /// \param[in] _pixmap The image to use as an greyed out icon.
      /// \param[in, out] _act Action that receives the icon.
      private: void CreateDisabledIcon(const std::string &_pixmap,
                   QAction *_act);

      /// \brief Callback when window mode has changed.
      /// \param[in] _mode Window mode, such as "Simulation", "LogPlayback"...
      private: void OnWindowMode(const std::string &_mode);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<MainWindowPrivate> dataPtr;
    };
  }
}
#endif
