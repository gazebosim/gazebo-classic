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
#ifndef _ALIGN_WIDGET_HH_
#define _ALIGN_WIDGET_HH_

#include <vector>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class AlignWidgetPrivate;

    /// \class AlignWidget AlignWidget.hh gui/Gui.hh
    /// \brief A gui widget for aligning models
    class GAZEBO_VISIBLE AlignWidget : public QWidget
    {
      Q_OBJECT

      public: enum AlignAxis
      {
        /// X
        ALIGN_X,
        /// Y
        ALIGN_Y,
        /// Z
        ALIGN_Z
      };

      public: enum AlignConfig
      {
        /// minimum
        ALIGN_MIN,
        /// center
        ALIGN_CENTER,
        /// max
        ALIGN_MAX
      };

      /// \brief Constructor
      /// \param[in] _parent Parent Qt widget.
      public: AlignWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~AlignWidget();

      /// \brief Initialize the alignment widget with Qt actions.
      /// \param[in] _xAlignActions a list of Qt actions for x alignment.
      /// \param[in] _yAlignActions a list of Qt actions for y alignment.
      /// \param[in] _zAlignActions a list of Qt actions for z alignment.
      public: void Add(AlignAxis _axis, QAction *_action,
          AlignConfig _mode);

      /// \brief Qt callback when a specific alignment configuration is
      /// triggered.
      private slots: void OnAlignMode(QString _mode);

      /// \brief Helper method to convert axis enum to string;
      /// \param[in] _axis input axis enum
      /// \return axis string
      private: std::string GetAxisAsString(AlignAxis _axis);

      /// \brief Helper method to convert mode enum to string;
      /// \param[in] _mode input mode enum
      /// \return mode string
      private: std::string GetModeAsString(AlignConfig _axis);

      /// \brief Qt event filter currently used for filtering enter and leave
      /// events.
      /// param[in] _obj Qt object watched by the event filter
      /// param[in] _event Qt event to be filtered.
      /// \return True to stop event propagation.
      private: bool eventFilter(QObject *_obj, QEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: AlignWidgetPrivate *dataPtr;
    };
  }
}
#endif
