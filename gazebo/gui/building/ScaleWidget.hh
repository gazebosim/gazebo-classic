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

#ifndef _RULER_WIDGET_HH_
#define _RULER_WIDGET_HH_

#include <string>
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class ScaleWidget ScaleWidget.hh
    /// \brief Widget that displays the scale (zoom level) of the editor
    class GZ_GUI_BUILDING_VISIBLE ScaleWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: ScaleWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~ScaleWidget();

      /// \brief Qt paint event for drawing the scale widget.
      /// \param[in] _event Qt paint event.
      private: void paintEvent(QPaintEvent *_event);

      /// \brief Callback received when the zoom level has changed.
      /// \param[in] _zoomFactor New zoom factor.
      private: void OnChangeZoom(double _zoomFactor);

      /// \brief Text displaying the scale.
      private: std::string scaleText;

      /// \brief A list of gui editor events connected to this widget.
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}

#endif
