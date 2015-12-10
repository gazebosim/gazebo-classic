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

#ifndef _GAZEBO_BUILDING_EDITOR_ITEM_PRIVATE_HH_
#define _GAZEBO_BUILDING_EDITOR_ITEM_PRIVATE_HH_

#include <string>

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the EditorItem class
    class EditorItemPrivate
    {
      /// \brief Z ordering of the rect item when idle (unselected.)
      public: int zValueIdle;

      /// \brief Z ordering of the rect item when selected.
      public: int zValueSelected;

      /// \brief Type of editor item.
      public: std::string editorType;

      /// \brief Name of editor item.
      public: std::string name;

      /// \brief Level that this item is on.
      public: int level;

      /// \brief Vertical distance from the building's base to the base of
      /// the level this editor is in.
      public: double levelBaseHeight;

      /// \brief Color of the associated 3D visual.
      public: QColor visual3dColor;

      /// \brief Texture of the associated 3D visual.
      public: QString visual3dTexture;

      /// \brief Transparency of the associated 3D visual.
      public: float visual3dTransparency;

      /// \brief Flag to indicate whether this item is currently highlighted or
      /// not.
      public: bool highlighted;

      /// \brief Scale for converting from pixel to metric units.
      public: double itemScale;
    };
  }
}
#endif
