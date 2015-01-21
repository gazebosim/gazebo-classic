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
#ifndef _IMPORT_IMAGE_DIALOG_HH_
#define _IMPORT_IMAGE_DIALOG_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class EditorView;
    class ImportImageView;

    /// \addtogroup gazebo_gui
    /// \{

    class GAZEBO_VISIBLE ImportImageDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: ImportImageDialog(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~ImportImageDialog();

      /// \brief A signal used to set the file label.
      /// \param[in] _string File name string.
      signals: void SetFileName(QString _string);

      /// \brief Qt callback when a file is selected.
      private slots: void OnSelectFile();

      /// \brief Qt callback when the dialog is accepted.
      private slots: void OnAccept();

      /// \brief Qt callback when the dialog is rejected.
      private slots: void OnReject();

      /// \brief Qt callback when the Next button is clicked.
      private slots: void OnNext();

      /// \brief Qt callback when the Back button is clicked.
      private slots: void OnBack();

      /// \brief Qt callback when the distance spin's value is changed.
      private slots: void OnChangeDistance(double _distance);

      /// \brief Qt callback when the resolution spin's value is changed.
      private slots: void OnChangeResolution(double _resolution);

      /// \brief Distance spin box.
      public: QDoubleSpinBox *distanceSpin;

      /// \brief Resolution spin box.
      public: QDoubleSpinBox *resolutionSpin;

      /// \brief Building editor 2D view.
      private: EditorView *view;

      /// \brief Stacked layout of steps 1 and 2.
      private: QStackedLayout *stackedStepLayout;

      /// \brief Next button.
      private: QPushButton *nextButton;

      /// \brief Ok button.
      private: QPushButton *okButton;

      /// \brief File path line edit.
      private: QLineEdit *fileLineEdit;

      /// \brief Import image view width.
      private: int imageDisplayWidth;

      /// \brief Import image view height.
      private: int imageDisplayHeight;

      /// \brief Import image view.
      private: ImportImageView *importImageView;

      /// \brief Point where measure line starts.
      private: QPointF measureLineStart;

      /// \brief Indicates whether currently drawing a line or not.
      private: bool drawingLine;
    };
    /// \}
  }
}

#endif
