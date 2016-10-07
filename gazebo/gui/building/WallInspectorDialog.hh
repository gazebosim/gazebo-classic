/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_BUILDING_WALLINSPECTORDIALOG_HH_
#define GAZEBO_GUI_BUILDING_WALLINSPECTORDIALOG_HH_

#include <memory>
#include <string>
#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/BaseInspectorDialog.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class WallInspectorDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class WallInspectorDialog WallInspectorDialog.hh
    /// \brief Dialog for configuring a wall item.
    class GZ_GUI_VISIBLE WallInspectorDialog
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
      public: double Length() const;

      /// \brief Get the start position of the wall segment.
      /// \return Wall segment start position in pixel coordinates.
      public: ignition::math::Vector2d StartPosition() const;

      /// \brief Get the end position of the wall segment.
      /// \return Wall segment end position in pixel coordinates.
      public: ignition::math::Vector2d EndPosition() const;

      /// \brief Get the height of the wall.
      /// \return Wall height in pixels.
      public: double Height() const;

      /// \brief Get the thickness of the wall.
      /// \return Wall thickness in pixels.
      public: double Thickness() const;

      /// \brief Set the name of the wall.
      /// \param[in] _name Name to set the wall to.
      public: void SetName(const std::string &_name);

      /// \brief Set the length of the wall segment.
      /// \param[in] _length Length of the wall segment in pixels.
      public: void SetLength(const double _length);

      /// \brief Set the start position of the wall segment.
      /// \param[in] _pos Start position of the wall segment in pixel
      /// coordinates.
      public: void SetStartPosition(const ignition::math::Vector2d &_pos);

      /// \brief Set the end position of the wall segment.
      /// \param[in] _pos end position of the wall segment in pixel coordinates.
      public: void SetEndPosition(const ignition::math::Vector2d &_pos);

      /// \brief Set the height of the wall.
      /// \param[in] _height Height of wall in pixels.
      public: void SetHeight(const double _height);

      /// \brief Set the thickness of the wall.
      /// \param[in] _thickness Thickness of wall in pixels.
      public: void SetThickness(const double _thickness);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<WallInspectorDialogPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
