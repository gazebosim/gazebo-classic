/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_PLOT_EDITABLELABEL_HH_
#define _GAZEBO_GUI_PLOT_EDITABLELABEL_HH_

#include <memory>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class EditableLabelPrivate;

    /// \brief Plot Curve data.
    class GZ_GUI_VISIBLE EditableLabel : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _label Label.
      /// \param[in] _parent Parent widget.
      public: EditableLabel(const std::string &_label, QWidget *_parent = NULL);

      /// \brief Destructor.
      public: ~EditableLabel();

      /// \brief Qt event when the focus is lost
      /// \param[in] _event Qt focus event
      // public: void focusOutEvent(QFocusEvent * event);

      /// \brief Qt mouse double click event.
      /// \param[in] _event Qt mouse event
      protected: virtual void mouseDoubleClickEvent(QMouseEvent *_event);

      /// \brief Qt key press event.
      /// \param[in] _event Qt key event
      protected: virtual void keyPressEvent(QKeyEvent *_event);

      /// \brief Qt callback when line edit loses focus
      private slots: void OnEditingFinished();

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<EditableLabelPrivate> dataPtr;
    };
  }
}
#endif
