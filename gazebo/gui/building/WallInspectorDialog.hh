/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"
#include "gazebo/gui/building/BaseInspectorDialog.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class WallInspectorDialog WallInspectorDialog.hh
    /// \brief Dialog for configuring a wall item.
    class GZ_GUI_BUILDING_VISIBLE WallInspectorDialog
      : public BaseInspectorDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: WallInspectorDialog(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~WallInspectorDialog();

      /// \brief Get the length the wall segment.
      /// \return Wall segment length in pixels.
      public: double GetLength() const;

      /// \brief Get the start position of the wall segment.
      /// \return Wall segment start position in pixel coordinates.
      public: QPointF GetStartPosition() const;

      /// \brief Get the end position of the wall segment.
      /// \return Wall segment end position in pixel coordinates.
      public: QPointF GetEndPosition() const;

      /// \brief Get the height of the wall.
      /// \return Wall height in pixels.
      public: double GetHeight() const;

      /// \brief Get the thickness of the wall.
      /// \return Wall thickness in pixels.
      public: double GetThickness() const;

      /// \brief Set the name of the wall.
      /// \param[in] _name Name to set the wall to.
      public: void SetName(const std::string &_name);

      /// \brief Set the length of the wall segment.
      /// \param[in] _length Length of the wall segment in pixels.
      public: void SetLength(double _length);

      /// \brief Set the start position of the wall segment.
      /// \param[in] _pos Start position of the wall segment in pixel
      /// coordinates.
      public: void SetStartPosition(const QPointF &_pos);

      /// \brief Set the end position of the wall segment.
      /// \param[in] _pos end position of the wall segment in pixel coordinates.
      public: void SetEndPosition(const QPointF &_pos);

      /// \brief Set the height of the wall.
      /// \param[in] _height Height of wall in pixels.
      public: void SetHeight(double _height);

      /// \brief Set the thickness of the wall.
      /// \param[in] _thickness Thickness of wall in pixels.
      public: void SetThickness(double _thickness);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApply();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Spin box for configuring the X start position of the wall
      /// segment.
      private: QDoubleSpinBox *startXSpinBox;

      /// \brief Spin box for configuring the Y start position of the wall
      /// segment.
      private: QDoubleSpinBox *startYSpinBox;

      /// \brief Spin box for configuring the X end position of the wall
      /// segment.
      private: QDoubleSpinBox *endXSpinBox;

      /// \brief Spin box for configuring the Y end position of the wall
      /// segment.
      private: QDoubleSpinBox *endYSpinBox;

      /// \brief Spin box for configuring the height of the wall.
      private: QDoubleSpinBox *heightSpinBox;

      /// \brief Spin box for configuring the thickness of the wall.
      private: QDoubleSpinBox *thicknessSpinBox;

      /// \brief Spin box for configuring the length of the wall segment.
      private: QDoubleSpinBox *lengthSpinBox;

      /// \brief Label that holds the name of the wall.
      private: QLabel* wallNameLabel;
    };
    /// \}
  }
}

#endif
