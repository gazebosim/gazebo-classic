/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _WALL_INSPECTOR_DIALOG_HH_
#define _WALL_INSPECTOR_DIALOG_HH_

#include <string>
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class WallInspectorDialog : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: WallInspectorDialog(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~WallInspectorDialog();

      public: double GetLength() const;

      public: QPointF GetStartPosition() const;

      public: QPointF GetEndPosition() const;

      public: double GetHeight() const;

      public: double GetThickness() const;

      public: std::string GetMaterial() const;

      public: void SetName(const std::string &_name);

      public: void SetLength(double _length);

      public: void SetStartPosition(const QPointF &_pos);

      public: void SetEndPosition(const QPointF &_pos);

      public: void SetHeight(double _height);

      public: void SetThickness(double _thickness);

      public: void SetMaterial(const std::string &_material);

      /// \brief Qt signal emitted to indicate that changes should be applied
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the Cancel button is pressed
      private slots: void OnCancel();

      /// \brief Qt callback when the Apply button is pressed
      private slots: void OnApply();

      /// \brief Qt callback when the Ok button is pressed
      private slots: void OnOK();

      private: QDoubleSpinBox *startXSpinBox;

      private: QDoubleSpinBox *startYSpinBox;

      private: QDoubleSpinBox *endXSpinBox;

      private: QDoubleSpinBox *endYSpinBox;

      private: QDoubleSpinBox *heightSpinBox;

      private: QDoubleSpinBox *thicknessSpinBox;

      private: QLabel* wallNameLabel;

      private: QDoubleSpinBox *lengthSpinBox;

      private: QComboBox *materialComboBox;
    };
  }
}

#endif
