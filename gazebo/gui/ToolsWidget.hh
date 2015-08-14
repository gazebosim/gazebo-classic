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
#ifndef _TOOLS_WIDGET_HH_
#define _TOOLS_WIDGET_HH_

#include <vector>
#include <string>

#include "gazebo/common/Event.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class JointControlWidget;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ToolsWidget ToolsWidget.hh gui/ToolsWidget.hh
    /// \brief A widget that manages all the tools on the right side of the
    /// render widget.
    class GZ_GUI_VISIBLE ToolsWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget pointer.
      public: ToolsWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~ToolsWidget();

      /// \brief Callback when an entity is selected.
      /// \param[in] _name Name of the selected entity.
      private: void OnSetSelectedEntity(const std::string &_name,
                                        const std::string &_mode);

      /// \brief Pointer to the tab widget for all the tools.
      private: QTabWidget *tabWidget;

      /// \brief Joint control tool
      private: JointControlWidget *jointControlWidget;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}
#endif
