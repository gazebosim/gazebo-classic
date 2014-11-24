/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _RENDER_WIDGET_HH_
#define _RENDER_WIDGET_HH_

#include <string>
#include <vector>
#include <sdf/sdf.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

class QLineEdit;
class QLabel;
class QFrame;
class QHBoxLayout;

namespace gazebo
{
  class GLWidget;

  namespace gui
  {
    class BuildingEditorWidget;

    class GAZEBO_VISIBLE RenderWidget : public QWidget
    {
      Q_OBJECT
      public: RenderWidget(QWidget *_parent = 0);
      public: virtual ~RenderWidget();

      public: void RemoveScene(const std::string &_name);
      public: void CreateScene(const std::string &_name);

      /// \brief Show editor widget in the main window
      /// param[in] _show True to show the editor widget, false to hide it.
      public: void ShowEditor(bool _show);

      /// \brief Display an overlay message
      /// \param[in] _msg Message to be displayed
      /// \param [in] _duration Duration in milliseconds
      public: void DisplayOverlayMsg(const std::string &_msg,
          int _duration = -1);

      /// \brief Get the overlay message being displayed
      /// \return Message displayed in the render window
      public: std::string GetOverlayMsg() const;

      /// \brief Add a plugin to the render widget.
      /// \param[in] _plugin Plugin pointer to add.
      /// \param[in] _elem Plugin sdf parameters.
      public: void AddPlugin(GUIPluginPtr _plugin, sdf::ElementPtr _elem);

      /// \brief Get the toolbar on top of the render widget
      /// \return Toolbar
      public: QToolBar *GetToolbar() const;

      /// \brief Set the visibility of the toolbar.
      /// \param[in] _show Whether or not to show the toolbar.
      public: void ShowToolbar(const bool _show);

      private slots: virtual void update();

      /// \brief Qt callback to clear overlay message if a duration is
      /// specified
      private slots: void OnClearOverlayMsg();

      private: void OnFullScreen(bool &_value);

      /// \brief Handle follow model user event.
      /// \param[in] _modelName Name of the model that is being followed.
      private: void OnFollow(const std::string &_modelName);

      /// \brief Handle align model user event.
      private: void OnAlign();

      /// \brief Widget used to draw the scene.
      private: GLWidget *glWidget;

      /// \brief Building editor widget for creating a building model
      private: BuildingEditorWidget *buildingEditorWidget;

      /// \brief Frame that holds the contents of this widget.
      private: QFrame *mainFrame;

      /// \brief All event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Bottom frame that holds the play/pause widgets
      private: QFrame *bottomFrame;
      private: QLabel *xyzLabel;
      private: QLineEdit *xPosEdit;
      private: QLineEdit *yPosEdit;
      private: QLineEdit *zPosEdit;

      private: QLabel *rpyLabel;
      private: QLineEdit *rollEdit;
      private: QLineEdit *pitchEdit;
      private: QLineEdit *yawEdit;
      private: QLineEdit *fpsEdit;
      private: QLineEdit *trianglesEdit;

      /// \brief Widget for the top toolbar
      private: QToolBar *toolbar;

      private: QToolBar *mouseToolbar;
      private: QToolBar *editToolbar;

      /// \brief An overlay label on the 3D render widget
      private: QLabel *msgOverlayLabel;

      private: bool clear;
      private: std::string clearName;

      private: bool create;
      private: std::string createName;
      private: QTimer *timer;

      /// \brief Base overlay message;
      private: std::string baseOverlayMsg;

      /// \brief All the gui plugins
      private: std::vector<gazebo::GUIPluginPtr> plugins;
    };
  }
}
#endif
