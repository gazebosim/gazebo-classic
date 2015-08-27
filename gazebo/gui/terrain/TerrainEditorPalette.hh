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

#ifndef _TERRAIN_EDITOR_PALETTE_HH_
#define _TERRAIN_EDITOR_PALETTE_HH_

#include <string>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    class Heightmap;
  }

  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class TerrainEditorPalette TerrainEditorPalette.hh
    /// \brief A palette of building items which can be added to the editor.
    class GZ_GUI_VISIBLE TerrainEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: TerrainEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~TerrainEditorPalette();

      /// \brief Mouse event filter callback when mouse button is pressed.
      /// \param[in] _event The mouse event.
      /// \return True if the brush was applied
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is moved.
      /// \param[in] _event The mouse event.
      /// \return True if the brush was applied
      private: bool OnMouseMove(const common::MouseEvent &_event);

      /// \brief Apply a brush
      /// \param[in] _event The mouse event.
      /// \param[in] _camera Active camera.
      /// \param[in] _heightmap Heightmap on which to apply the modification.
      /// \return True if the brush was applied
      private: bool Apply(const common::MouseEvent &_event,
                   rendering::CameraPtr _camera,
                   rendering::Heightmap *_heightmap);

      /// \brief Set the current state.
      /// \param[in] _state String state(lower, raise, flatten ...). Empty
      /// string to disable terrain modification.
      private: void SetState(const std::string &_state);

      /// \brief QT callback when Raise button is toggled.
      /// \param[in] _toggle True if the button is enabled.
      private slots: void OnRaise(bool _toggle);

      /// \brief QT callback when Lower button is toggled.
      /// \param[in] _toggle True if the button is enabled.
      private slots: void OnLower(bool _toggle);

      /// \brief QT callback when Flatten button is toggled.
      /// \param[in] _toggle True if the button is enabled.
      private slots: void OnFlatten(bool _toggle);

      /// \brief QT callback when pick height button is toggled.
      /// \param[in] _toggle True if the button is enabled.
      private slots: void OnPickHeight(bool _toggle);

      /// \brief QT callback when image is saved.
      private slots: void OnSave();

      /// \brief QT callback, when outside radius spin box is modified.
      /// \param[in] _value The new value.
      private slots: void OnOutsideRadiusSpin(double _value);

      /// \brief QT callback, when outside radius slider is modified.
      /// \param[in] _value The new value.
      private slots: void OnOutsideRadiusSlider(int _value);

      /// \brief QT callback, when inside radius spin box is modified.
      /// \param[in] _value The new value.
      private slots: void OnInsideRadiusSpin(double _value);

      /// \brief QT callback, when inside radius slider is modified.
      /// \param[in] _value The new value.
      private slots: void OnInsideRadiusSlider(int _value);

      /// \brief QT callback, when weight spin box is modified.
      /// \param[in] _value The new value.
      private slots: void OnWeightSpin(double _value);

      /// \brief QT callback, when weight slider is modified.
      /// \param[in] _value The new value.
      private slots: void OnWeightSlider(int _value);

      /// \brief QT callback, when height spin box is modified.
      /// \param[in] _value The new value.
      private slots: void OnHeightSpin(double _value);

      /// \brief QT callback, when height slider is modified.
      /// \param[in] _value The new value.
      private slots: void OnHeightSlider(int _value);

      /// \brief Spin to control the outside size of the brush.
      private: QDoubleSpinBox *outsideRadiusSpin;

      /// \brief Slider to control the outside size of the brush.
      private: QSlider *outsideRadiusSlider;

      /// \brief Spin to control the inside size of the brush.
      private: QDoubleSpinBox *insideRadiusSpin;

      /// \brief Slider to control the inside size of the brush.
      private: QSlider *insideRadiusSlider;

      /// \brief Spin to control the weight of the brush.
      private: QDoubleSpinBox *weightSpin;

      /// \brief Slider to control the weight of the brush.
      private: QSlider *weightSlider;

      /// \brief Spin to control the weight of the brush.
      private: QDoubleSpinBox *heightSpin;

      /// \brief Slider to control the weight of the brush.
      private: QSlider *heightSlider;

      /// \brief The current brush state.
      private: std::string state;
    };
  }
}
#endif
