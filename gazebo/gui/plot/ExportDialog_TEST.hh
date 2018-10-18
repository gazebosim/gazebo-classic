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
#ifndef GAZEBO_GUI_PLOT_EXPORTDIALOG_TEST_HH_
#define GAZEBO_GUI_PLOT_EXPORTDIALOG_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

namespace gazebo
{
  namespace gui
  {
    class ExportDialog;
  }
}

/// \brief A test class for the ExportDialog widget.
class ExportDialog_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Verify that the buttons in the export dialog are disabled.
  /// \param[in] _exportDialog Pointer to the export dialog.
  /// \param[in] _enabled Enabled value the Export button should have.
  private: void VerifyButtons(gazebo::gui::ExportDialog *_exportDialog,
               const bool _enabled);

  /// \brief Trigger the select all or clear selection buttons.
  /// \param[in] _exportDialog Pointer to the export dialog.
  /// \param[in] _all True to select all, false to clear selection.
  private: void Select(gazebo::gui::ExportDialog *_exportDialog,
               const bool _all);

  /// \brief Test adding and removing plots
  private slots: void Empty();

  /// \brief Test adding and removing plots
  private slots: void OnePlot();

  /// \brief Export a plot to PDF
  private slots: void ExportPDF();

  /// \brief Export a plot to CSV
  private slots: void ExportCSV();
};
#endif
