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
#ifndef _EDITOR_HH_
#define _EDITOR_HH_

#include <string>
#include <vector>

#include "gazebo/gui/qt.h"
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class MainWindow;

    /// \brief Base class for editors, such as BuildingEditor and
    /// TerrainEditor.
    class GAZEBO_VISIBLE Editor : public QObject
    {
      Q_OBJECT

      /// \brief Constuctor.
      /// \param[in] _mainWindow Pointer to the main window.
      public: Editor(MainWindow *_mainWindow);

      /// \brief Destuctor
      public: virtual ~Editor();

      /// \brief Initialize the editor. Each child class should call this
      /// function on construction.
      /// \param[in] _objName Name of the object, which can be used in the
      /// style sheet.
      /// \param[in] _tabLabel String used for the tab label.
      /// \param[in] _widget Widget that is put inside the tab.
      /// \param[in] _cornerWidget Optional widget to go on the top right
      /// corner.
      protected: void Init(const std::string &_objName,
          const std::string &_tabLabel, QWidget *_widget,
          QWidget *_cornerWidget = NULL);

      /// \brief The tab widget that holds the editor's set of buttons.
      protected: QTabWidget *tabWidget;

      /// \brief Pointer to the main window.
      protected: MainWindow *mainWindow;

      /// \brief List of Event based connections.
      protected: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
