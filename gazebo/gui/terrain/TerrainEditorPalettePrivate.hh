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
#ifndef _GAZEBO_TERRAIN_EDITOR_PALETTE_PRIVATE_HH_
#define _GAZEBO_TERRAIN_EDITOR_PALETTE_PRIVATE_HH_

#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/ApplyWrenchDialog.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class TerrainEditorPalettePrivate TerrainEditorPalettePrivate.hh
    /// \brief Private data for the TerrainEditorPalette class.
    class TerrainEditorPalettePrivate
    {
      /// \brief Spin to control the outside size of the brush.
      public: QDoubleSpinBox *outsideRadiusSpin;

      /// \brief Slider to control the outside size of the brush.
      public: QSlider *outsideRadiusSlider;

      /// \brief Spin to control the inside size of the brush.
      public: QDoubleSpinBox *insideRadiusSpin;

      /// \brief Slider to control the inside size of the brush.
      public: QSlider *insideRadiusSlider;

      /// \brief Spin to control the weight of the brush.
      public: QDoubleSpinBox *weightSpin;

      /// \brief Slider to control the weight of the brush.
      public: QSlider *weightSlider;

      /// \brief Spin to control the weight of the brush.
      public: QDoubleSpinBox *heightSpin;

      /// \brief Slider to control the weight of the brush.
      public: QSlider *heightSlider;

      /// \brief The current brush state.
      public: std::string state;
    };
  }
}
#endif
