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
#ifndef GAZEBO_GUI_BUILDING_SCALEWIDGET_HH_
#define GAZEBO_GUI_BUILDING_SCALEWIDGET_HH_

#include <memory>

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class ScaleWidgetPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ScaleWidget ScaleWidget.hh
    /// \brief Widget that displays the scale (zoom level) of the editor
    class GZ_GUI_VISIBLE ScaleWidget : public QWidget
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
      private: void OnChangeZoom(const double _zoomFactor);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ScaleWidgetPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
