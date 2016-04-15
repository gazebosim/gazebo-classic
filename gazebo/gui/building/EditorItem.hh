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

#ifndef _GAZEBO_GUI_BUILDING_EDITORITEM_HH_
#define _GAZEBO_GUI_BUILDING_EDITORITEM_HH_

#include <string>
#include <memory>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Color.hh"

#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data.
    class EditorItemPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class EditorItem EditorItem.hh
    /// \brief Base class of an item in the editor.
    class GZ_GUI_VISIBLE EditorItem : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: EditorItem();

      /// \brief Destructor
      public: ~EditorItem();

      /// \brief Get the size of the item in pixels.
      /// \return Size in pixels.
      public: virtual ignition::math::Vector3d Size() const;

      /// \brief Get the scene position of editor item.
      /// \return Scene position in pixel coordinates.
      public: virtual ignition::math::Vector3d ScenePosition() const;

      /// \brief Get the scene rotation of the editor item.
      /// \return Scene rotation in degrees.
      public: virtual double SceneRotation() const;

      /// \brief Get the type of the editor item.
      /// \return Type of the item.
      public: virtual std::string ItemType() const;

      /// \brief Get the name of the editor item.
      /// \return Name of the item.
      public: virtual std::string Name() const;

      /// \brief Get the level in which this building item is located.
      public: int Level() const;

      /// \brief Get the base height of this level relative to the ground
      /// plane.
      public: double LevelBaseHeight() const;

      /// \brief Get the associated 3D visual's color.
      /// \return Color of the 3D visual.
      public: virtual common::Color Color3d() const;

      /// \brief Get the associated 3D visual's texture.
      /// \return Texture of the 3D visual.
      public: virtual std::string Texture3d() const;

      /// \brief Set the name of this editor item.
      /// \param[in] _name Name to set the editor item to.
      public: virtual void SetName(const std::string &_name);

      /// \brief Set the level of this building item.
      /// \param[in] _level Level number.
      public: void SetLevel(const int _level);

      /// \brief Set the base height of this level relative to the ground
      /// plane.
      /// \param[in] _height Base height.
      public: void SetLevelBaseHeight(const double _height);

      /// \brief Set the associated 3D visual's color.
      /// \param[in] _color Color.
      public: void SetColor3d(const common::Color &_color);

      /// \brief Set the associated 3D visual's texture.
      /// \param[in] _texture Texture.
      public: void SetTexture3d(const std::string &_texture);

      /// \brief Set the transparency of the associated 3D visual.
      /// \param[in] _transparency Transparency.
      public: void Set3dTransparency(const float _transparency);

      /// \brief Set whether this item should be highlighted or not.
      /// \param[in] _highlighted True for highlighted.
      public: virtual void SetHighlighted(const bool _highlighted);

      /// \brief Get the z value of this item when in idle state.
      /// \return Z value.
      public: int ZValueIdle() const;

      /// \brief Get the z value of this item when in selected state.
      /// \return Z value.
      public: int ZValueSelected() const;

      /// \brief Qt signal emitted when the editor item size has changed.
      /// \param[in] _width Width of item in pixels.
      /// \param[in] _depth Depth of item in pixels.
      /// \param[in] _height Height of item in pixels.
      Q_SIGNALS: void SizeChanged(const double _width, const double _depth,
          const double _height);

      /// \brief Qt signal emitted when the editor item pose has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void PoseChanged(const double _x, const double _y,
          const double _z, const double _roll, const double _pitch,
          const double _yaw);

      /// \brief Qt signal emitted when the editor item pose origin has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void PoseOriginTransformed(const double _x, const double _y,
          const double _z, const double _roll, const double _pitch,
          const double _yaw);

      /// \brief Qt signal emitted when the editor item position has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      Q_SIGNALS: void PositionChanged(const double _x, const double _y,
          const double _z);

      /// \brief Qt signal emitted when the editor item rotation has changed.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void RotationChanged(const double _roll, const double _pitch,
          const double _yaw);

      /// \brief Qt signal emitted when the editor item width has changed.
      /// \param[in] _width Width of item in pixels.
      Q_SIGNALS: void WidthChanged(const double _width);

      /// \brief Qt signal emitted when the editor item depth has changed.
      /// \param[in] _depth Depth of item in pixels.
      Q_SIGNALS: void DepthChanged(const double _depth);

      /// \brief Qt signal emitted when the editor item height has changed.
      /// \param[in] _height Height of item in pixels.
      Q_SIGNALS: void HeightChanged(const double _height);

      /// \brief Qt signal emitted when the editor item's X position has
      /// changed.
      /// \param[in] _x X position of item in pixels.
      Q_SIGNALS: void PosXChanged(const double _posX);

      /// \brief Qt signal emitted when the editor item's Y position has
      /// changed.
      /// \param[in] _y Y position of item in pixels.
      Q_SIGNALS: void PosYChanged(const double _posY);

      /// \brief Qt signal emitted when the editor item's Z position has
      /// changed.
      /// \param[in] _z Z position of item in pixels.
      Q_SIGNALS: void PosZChanged(const double _posZ);

      /// \brief Qt signal emitted when the editor item yaw rotation has
      /// changed.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void YawChanged(const double _yaw);

      /// \brief Qt signal emitted when the editor item's level has changed.
      /// \param[in] _level Level.
      Q_SIGNALS: void LevelChanged(const int _level);

      /// \brief Qt signal emitted when the editor item's 3D color has
      /// changed.
      /// \param[in] _color Color.
      Q_SIGNALS: void ColorChanged(const common::Color &_color);

      /// \brief Qt signal emitted when the editor item's 3D texture has
      /// changed.
      /// \param[in] _texture Texture.
      Q_SIGNALS: void TextureChanged(const std::string &_texture);

      /// \brief Qt signal emitted when the editor item's 3D transparency has
      /// changed.
      /// \param[in] _transparency Transparency.
      Q_SIGNALS: void TransparencyChanged(const float _transparency);

      /// \brief Qt signal emitted when the editor item is being deleted.
      Q_SIGNALS: void ItemDeleted();

      /// \brief Qt callback when the color has been changed from the 3D view.
      /// \param[in] _color Color.
      private slots: void OnColorChanged(const common::Color &_color);

      /// \brief Qt callback when the texture has been changed from the 3D view.
      /// \param[in] _texture Texture.
      private slots: void OnTextureChanged(const std::string &_texture);

      /// \brief Z ordering of the rect item when idle (unselected.)
      protected: int zValueIdle;

      /// \brief Z ordering of the rect item when selected.
      protected: int zValueSelected;

      /// \brief Type of editor item.
      protected: std::string editorType;

      /// \brief Name of editor item.
      protected: std::string name;

      /// \brief Level that this item is on.
      protected: int level;

      /// \brief Vertical distance from the building's base to the base of
      /// the level this editor is in.
      protected: double levelBaseHeight;

      /// \brief Color of the associated 3D visual.
      protected: common::Color visual3dColor;

      /// \brief Texture of the associated 3D visual.
      protected: std::string visual3dTexture;

      /// \brief Transparency of the associated 3D visual.
      protected: float visual3dTransparency;

      /// \brief Flag to indicate whether this item is currently highlighted or
      /// not.
      protected: bool highlighted;

      /// \brief Scale for converting from pixel to metric units.
      protected: double itemScale;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<EditorItemPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
