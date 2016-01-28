/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_GUI_IMPORT_IMAGE_DIALOG_PRIVATE_HH_
#define _GAZEBO_GUI_IMPORT_IMAGE_DIALOG_PRIVATE_HH_

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    // Forward declare pointers
    class EditorView;
    class ImportImageView;

    /// \internal
    /// \brief Private data for ImportImageDialog
    class ImportImageDialogPrivate
    {
      /// \brief Distance spin box.
      public: QDoubleSpinBox *distanceSpin;

      /// \brief Resolution spin box.
      public: QDoubleSpinBox *resolutionSpin;

      /// \brief Building editor 2D view.
      public: EditorView *view;

      /// \brief Stacked layout of steps 1 and 2.
      public: QStackedLayout *stackedStepLayout;

      /// \brief Next button.
      public: QPushButton *nextButton;

      /// \brief Ok button.
      public: QPushButton *okButton;

      /// \brief File path line edit.
      public: QLineEdit *fileLineEdit;

      /// \brief Import image view width.
      public: int imageDisplayWidth;

      /// \brief Import image view height.
      public: int imageDisplayHeight;

      /// \brief Import image view.
      public: ImportImageView *importImageView;

      /// \brief Indicates whether currently drawing a line or not.
      public: bool drawingLine;
    };
  }
}

#endif
