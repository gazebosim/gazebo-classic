/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_BUILDING_IMPORTIMAGEDIALOG_HH_
#define GAZEBO_GUI_BUILDING_IMPORTIMAGEDIALOG_HH_

#include <memory>

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class ImportImageDialogPrivate;
    // Forward declare friend class.
    class ImportImageView;

    /// \addtogroup gazebo_gui
    /// \{
    class GZ_GUI_VISIBLE ImportImageDialog : public QDialog
    {
      friend class ImportImageView;

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

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ImportImageDialogPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
