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

#ifndef _GAZEBO_GUI_PLOT_EXPORTDIALOG_HH_
#define _GAZEBO_GUI_PLOT_EXPORTDIALOG_HH_

#include <string>
#include <memory>
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class ExportDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ExportDialog gui/ExportDialog.hh
    /// \brief Dialog for saving to file.
    class ExportDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: ExportDialog(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~ExportDialog();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the import button is pressed.
      private slots: void OnExport();

      /// \brief Qt callback when a plot icon is selected.
      private slots: void OnSelected(const QModelIndex &_index);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<ExportDialogPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
