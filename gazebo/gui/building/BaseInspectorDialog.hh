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
#ifndef GAZEBO_GUI_BUILDING_BASEINSPECTORDIALOG_HH_
#define GAZEBO_GUI_BUILDING_BASEINSPECTORDIALOG_HH_

#include <memory>
#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class Color;
  }

  namespace gui
  {
    // Forward declare private data.
    class BaseInspectorDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class BaseInspectorDialog BaseInspectorDialog.hh
    /// \brief Base Dialog for a specific inspector dialog.
    class GZ_GUI_VISIBLE BaseInspectorDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget
      public: BaseInspectorDialog(QWidget *_parent);

      /// \brief Destructor
      public: ~BaseInspectorDialog();

      /// \brief Initiate color combo box.
      public: void InitColorComboBox();

      /// \brief Initiate texture combo box.
      public: void InitTextureComboBox();

      /// \brief Get the color.
      /// \return Color.
      public: common::Color Color() const;

      /// \brief Get the texture.
      /// \return Texture.
      public: std::string Texture() const;

      /// \brief Set the color.
      /// \param[in] _color Color.
      public: void SetColor(const common::Color &_color);

      /// \brief Set the texture.
      /// \param[in] _texture Texture.
      public: void SetTexture(const std::string &_texture);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the Cancel button is pressed.
      protected slots: void OnCancel();

      /// \brief Qt callback when the Apply button is pressed.
      protected slots: void OnApply();

      /// \brief Qt callback when the Ok button is pressed.
      protected slots: void OnOK();

      /// \brief Combo box for selecting the color.
      protected: QComboBox *colorComboBox;

      /// \brief Combo box for selecting the texture.
      protected: QComboBox *textureComboBox;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<BaseInspectorDialogPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
