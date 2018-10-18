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

#ifndef _GAZEBO_EXTRUDE_DIALOG_HH_
#define _GAZEBO_EXTRUDE_DIALOG_HH_

#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class ExtrudeDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ExtrudeDialog ExtrudeDialog.hh gui/gui.hh
    /// \brief Dialog for saving to file.
    class GZ_GUI_VISIBLE ExtrudeDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _filename Full path to the SVG file.
      /// \param[in] _parent Parent QWidget.
      public: ExtrudeDialog(const std::string &_filename, QWidget *_parent = 0);

      /// \brief Destructor.
      public: ~ExtrudeDialog();

      /// \brief Reload SVG file to update the view with the current data in
      /// the dialog.
      public: void UpdateView();

      /// \brief Get the thickness value.
      /// \return Thickness value.
      public: double GetThickness() const;

      /// \brief Get the number of samples per segment.
      /// \return Number of samples.
      public: unsigned int GetSamples() const;

      /// \brief Get the resolution in px/m.
      /// \return Resolution value.
      public: double GetResolution() const;

      /// \brief Qt callback when the dialog is accepted.
      private slots: void OnAccept();

      /// \brief Qt callback when the dialog is rejected.
      private slots: void OnReject();

      /// \brief Qt callback when an int value was changed and the view.
      /// should be updated.
      /// \param[in] _value New value, not used but needed for the slot.
      private slots: void OnUpdateView(int _value);

      /// \brief Qt callback when a double value was changed and the view.
      /// should be updated.
      /// \param[in] _value New value, not used but needed for the slot.
      private slots: void OnUpdateView(double _value);

      /// \brief Qt event filter used to filter child widget events.
      /// \param[in] _obj Object that is watched by the event filter.
      /// \param[in] _event Qt event.
      /// \return True if the event is handled.
      private: bool eventFilter(QObject *_obj, QEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: ExtrudeDialogPrivate *dataPtr;
    };
    /// \}
  }
}

#endif
