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

#ifndef _SAVEDIALOG_TEST_HH_
#define _SAVEDIALOG_TEST_HH_

#include <string>
#include "gazebo/gui/QTestFixture.hh"


/// \brief A test class for the SaveDialog.
class SaveDialog_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test setting save location
  private slots: void SaveLocation();
};


namespace gazebo
{
  namespace gui
  {
    class SaveDialog;
  }
}

/// \brief A helper class for testing the SaveDialog.
class SaveDialogTestHelper : public QObject
{
  Q_OBJECT

  /// \brief Pointer to a SaveDiag
  public: gazebo::gui::SaveDialog *dialog;

  /// \brief Verify file dialog is working.
  private slots: void CheckFileDialog();
};


#endif
