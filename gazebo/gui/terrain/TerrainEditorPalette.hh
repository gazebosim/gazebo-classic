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

#ifndef _TERRAIN_EDITOR_PALETTE_HH_
#define _TERRAIN_EDITOR_PALETTE_HH_

#include <string>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/gui/qt.h"

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
    class TerrainEditorPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: TerrainEditorPalette(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~TerrainEditorPalette();

      /// \param[in] _event The mouse event.
      /// \return True if the brush was applied
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \param[in] _event The mouse event.
      /// \return True if the brush was applied
      private: bool OnMouseMove(const common::MouseEvent &_event);

      /// \brief Apply a brush
      /// \param[in] _event The mouse event.
      /// \return True if the brush was applied
      private: bool Apply(const common::MouseEvent &_event,
                   rendering::CameraPtr _camera,
                   rendering::Heightmap *_heightmap);

      /// \brief Add mouse event filters
      private: void AddEventFilters();

      /// \brief Remove mouse event filters
      private: void RemoveEventFilters();

      private slots: void OnRaise(bool _toggle);

      private slots: void OnLower(bool _toggle);

      private slots: void OnSmooth(bool _toggle);

      private slots: void OnSave();

      private: QSlider *brushSizeSlider;
      private: QSlider *brushWeightSlider;
      private: std::string state;
    };
  }
}
#endif
