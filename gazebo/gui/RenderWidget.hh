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
#ifndef _GAZEBO_RENDER_WIDGET_HH_
#define _GAZEBO_RENDER_WIDGET_HH_

#include <string>
#include <vector>
#include <sdf/sdf.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/TopToolbar.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"


namespace gazebo
{
  namespace gui
  {
    class GLWidget;
    class TimePanel;

    class GAZEBO_VISIBLE RenderWidget : public QWidget
    {
      Q_OBJECT
      public: RenderWidget(QWidget *_parent = 0);
      public: virtual ~RenderWidget();

      public: void RemoveScene(const std::string &_name);
      public: void CreateScene(const std::string &_name);

      /// \brief Add a widget inside the render widget
      /// \param[in] _widget Widget to be added.
      /// \param[in] _index Index in the splitter to add the widget at.
      public: void InsertWidget(unsigned int _index, QWidget *_widget);

      /// \brief Show the time panel.
      /// \para[in] _show True to show the panel, false to hide it.
      public: void ShowTimePanel(bool _show);

      /// \brief Get the time panel widget.
      /// \return the time panel widget.
      public: TimePanel *GetTimePanel() const;

      /// \brief Display an overlay message
      /// \param[in] _msg Message to be displayed
      /// \param [in] _duration Duration in milliseconds
      public: void DisplayOverlayMsg(const std::string &_msg,
          int _duration = -1);

      /// \brief Show or hide all the GUI overlays.
      /// \param[in] _visible True to show the GUI overlays, false to hide them.
      public: void SetOverlaysVisible(const bool _visible);

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

      /// \brief Qt callback to clear overlay message if a duration is
      /// specified
      private slots: void OnClearOverlayMsg();

      private: void OnFullScreen(bool &_value);

      /// \brief Handle follow model user event.
      /// \param[in] _modelName Name of the model that is being followed.
      private: void OnFollow(const std::string &_modelName);

      /// \brief Widget used to draw the scene.
      private: GLWidget *glWidget;

      /// \brief Frame that holds the contents of this widget.
      private: QFrame *mainFrame;

      /// \brief All event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Bottom frame that holds the play/pause widgets
      private: QFrame *bottomFrame;

      /// \brief Frame which holds the top toolbar.
      private: TopToolbar *topToolbar;

      /// \brief An overlay label on the 3D render widget
      private: QLabel *msgOverlayLabel;

      /// \brief Base overlay message;
      private: std::string baseOverlayMsg;

      /// \brief Vertical splitter between widgets.
      private: QSplitter *splitter;

      /// \brief All the gui plugins
      private: std::vector<gazebo::GUIPluginPtr> plugins;

      /// \brief Time panel widget.
      private: TimePanel *timePanel;
    };
  }
}
#endif
