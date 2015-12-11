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

#ifndef _GAZEBO_DIAGNOSTICS_HH_
#define _GAZEBO_DIAGNOSTICS_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class DiagnosticsPrivate;

    /// \brief Plot diagnostic information
    class GZ_GUI_VISIBLE Diagnostics : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Pointer to the parent widget.
      public: Diagnostics(QWidget *_parent);

      /// \brief Destructor.
      public: virtual ~Diagnostics();

      /// \brief Used to filter scroll wheel events.
      /// \param[in] _o Object that receives the event.
      /// \param[in] _event Pointer to the event.
      public: virtual bool eventFilter(QObject *_o, QEvent *_e);

      /// \brief Called when a diagnostic message is received.
      /// \param[in] _msg Diagnostic message.
      private: void OnMsg(ConstDiagnosticsPtr &_msg);

      /// \brief Update plots.
      private slots: void Update();

      /// \brief QT callback for the pause check button.
      /// \param[in] _value True when paused.
      private slots: void OnPause(bool _value);

      /// \brief Qt Callback when a plot should be added.
      private slots: void OnAddPlot();

      /// \internal
      /// \brief Pointer to private data.
      protected: std::unique_ptr<DiagnosticsPrivate> dataPtr;
    };
  }
}
#endif
