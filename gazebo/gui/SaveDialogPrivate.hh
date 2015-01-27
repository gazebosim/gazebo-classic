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
#ifndef _SAVE_DIALOG_PRIVATE_HH_
#define _SAVE_DIALOG_PRIVATE_HH_

#include <sdf/sdf.hh>

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \class SaveDialogPrivate SaveDialogPrivate.hh
    /// \brief Private data for the SaveDialog class
    class SaveDialogPrivate
    {
      /// \brief Widget container to hold advanced model saving options.
      public: QWidget *advancedOptionsWidget;

      /// \brief Label appearing at the top of the dialog box.
      public: QLabel *messageLabel;

      /// \brief Editable line that holds the model name.
      public: QLineEdit* modelNameLineEdit;

      /// \brief Editable line that holds the model's version.
      public: QLineEdit* modelVersionLineEdit;

      /// \brief Editable line that holds the model's description.
      public: QLineEdit* modelDescriptionLineEdit;

      /// \brief Editable line that holds the model's author's name.
      public: QLineEdit* modelAuthorNameLineEdit;

      /// \brief Editable line that holds the model's author's email.
      public: QLineEdit* modelAuthorEmailLineEdit;

      /// \brief Editable line that holds the model's save location.
      public: QLineEdit* modelLocationLineEdit;

      /// \brief The model's config file.
      public: TiXmlDocument modelConfig;
    };
  }
}
#endif
