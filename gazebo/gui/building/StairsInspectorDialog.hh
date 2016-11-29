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
#ifndef GAZEBO_GUI_BUILDING_STAIRSINSPECTORDIALOG_HH_
#define GAZEBO_GUI_BUILDING_STAIRSINSPECTORDIALOG_HH_

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
    class StairsInspectorDialogPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class StairsInspectorDialog StairsInspectorDialog.hh
    /// \brief Dialog for configuring a staircase item.
    class GZ_GUI_VISIBLE StairsInspectorDialog
      : public BaseInspectorDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: StairsInspectorDialog(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~StairsInspectorDialog();

      /// \brief Get start position (bottom-end) of the staircase.
      /// \return The start position of the staircase in pixel coordinates.
      public: ignition::math::Vector2d StartPosition() const;

      /// \brief Get the width the staircase.
      /// \return Staircase width in pixels.
      public: double Width() const;

      /// \brief Get the depth the staircase.
      /// \return Staircase depth in pixels.
      public: double Depth() const;

      /// \brief Get the height the staircase.
      /// \return Staircase height in pixels.
      public: double Height() const;

      /// \brief Get the number of steps in the staircase.
      /// \return Number of steps in the staircase.
      public: int Steps() const;

      /// \brief Set the name of the staircase.
      /// \param[in] _name Name to set to.
      public: void SetName(const std::string &_name);

      /// \brief Set the start position of the staircase.
      /// \param[in] _pos Start position in pixel coordinates.
      public: void SetStartPosition(const ignition::math::Vector2d &_pos);

      /// \brief Set the width of the staircase.
      /// \param[in] _width Width in pixels.
      public: void SetWidth(const double _width);

      /// \brief Set the depth of the staircase.
      /// \param[in] _depth Depth in pixels.
      public: void SetDepth(const double _depth);

      /// \brief Set the height of the staircase.
      /// \param[in] _height Height in pixels.
      public: void SetHeight(const double _height);

      /// \brief Set the number of steps in the staircase.
      /// \param[in] _steps Number of steps.
      public: void SetSteps(const int _steps);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<StairsInspectorDialogPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
