/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_HOTKEY_DIALOG_HH_
#define _GAZEBO_HOTKEY_DIALOG_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \brief Dialog displaying the keyboard shortcuts.
    class GAZEBO_VISIBLE HotkeyDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Parent QWidget.
      public: HotkeyDialog(QWidget *_parent = 0);

      /// \brief Qt callback when a link is clicked on the dialog's QWebView
      /// \param[in] Clicked url
      private slots: void OnLinkClicked(QUrl _url);
    };
    /// \}
  }
}

#endif
