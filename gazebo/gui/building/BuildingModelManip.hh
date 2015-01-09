/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _BUILDING_MODEL_MANIP_HH_
#define _BUILDING_MODEL_MANIP_HH_

#include <string>
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/common/Color.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class BuildingMaker;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingModelManip BuildingModelManip.hh
    /// \brief Manipulate a 3D visual associated to a 2D editor item.
    class GAZEBO_VISIBLE BuildingModelManip : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: BuildingModelManip();

      /// \brief Destructor
      public: virtual ~BuildingModelManip();

      /// \brief Get the name of the manip object.
      /// \return Name of the manip object.
      public: std::string GetName() const;

      /// \brief Get the visual this manip manages.
      /// \return A pointer to the visual object.
      public: rendering::VisualPtr GetVisual() const;

      /// \brief Get the transparency of the manip.
      /// \return Transparency.
      public: double GetTransparency() const;

      /// \brief Get the color of the manip.
      /// \return Color.
      public: common::Color GetColor() const;

      /// \brief Get the texture of the manip.
      /// \return Texture.
      public: std::string GetTexture() const;

      /// \brief Set the name of the manip object.
      /// \param[in] _name Name to set the manip to.
      public: void SetName(const std::string &_name);

      /// \brief Set the visual this manip manages.
      /// \param[in] _visual A pointer of the visual object.
      public: void SetVisual(const rendering::VisualPtr &_visual);

      /// \brief Set the maker that the manip is managed by.
      /// \param[in] _maker Maker that manages the manip.
      public: void SetMaker(BuildingMaker *_maker);

      /// \brief Get the parent of this manip.
      /// \return Parent manip.
      public: BuildingModelManip *GetParent() const;

      /// \brief Attach a manip as a child to this manip.
      /// \param[in] _manip manip to be attached.
      public: void AttachManip(BuildingModelManip *_manip);

      /// \brief Detach a child manip from this manip.
      /// \param[in] _manip manip to be detached.
      public: void DetachManip(BuildingModelManip *_manip);

      /// \brief Set the parent manip of this manip.
      /// \param[in] _parent Parent manip
      public: void SetAttachedTo(BuildingModelManip *_parent);

      /// \brief Detach this manip from its parent.
      public: void DetachFromParent();

      /// \brief Get a child manip by index.
      /// \param[in] _index Index of the child manip.
      /// \return The attached manip at index _index.
      public: BuildingModelManip *GetAttachedManip(unsigned int _index) const;

      /// \brief Get the number of child manips attached to this
      /// manip.
      /// \param[in] _index Index of the child manip.
      /// \return The number of attached manips.
      public: unsigned int GetAttachedManipCount() const;

      /// \brief Get whether or not this manip is attached to another.
      /// \return True if attached, false otherwise.
      public: bool IsAttached() const;

      /// \brief Set the pose of the manip.
      /// \param[in] _x X position in pixel coordinates.
      /// \param[in] _y Y position in pixel coordinates.
      /// \param[in] _z Z position in pixel coordinates.
      /// \param[in] _roll Roll rotation in degrees.
      /// \param[in] _pitch Pitch rotation in degrees.
      /// \param[in] _yaw Yaw rotation in degrees.
      public: void SetPose(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \brief Set the position of the manip.
      /// \param[in] _x X position in pixel coordinates.
      /// \param[in] _y Y position in pixel coordinates.
      /// \param[in] _z Z position in pixel coordinates.
      public: void SetPosition(double _x, double _y, double _z);

      /// \brief Set the rotation of the manip.
      /// \param[in] _roll Roll rotation in degrees.
      /// \param[in] _pitch Pitch rotation in degrees.
      /// \param[in] _yaw Yaw rotation in degrees.
      public: void SetRotation(double _roll, double _pitch, double _yaw);

      /// \brief Set the size of the manip.
      /// \param[in] _width Width in pixels.
      /// \param[in] _depth Depth in pixels.
      /// \param[in] _height Height pixels.
      public: void SetSize(double _width, double _depth, double _height);

      /// \brief Set the color of the manip.
      /// \param[in] _color Color.
      public: void SetColor(QColor _color);

      /// \brief Set the texture of the manip.
      /// \param[in] _texture Texture.
      public: void SetTexture(QString _texture);

      /// \brief Set the transparency of the manip.
      /// \param[in] _transparency Transparency.
      public: void SetTransparency(float _transparency);

      /// \brief Set the visibility of the manip.
      /// \param[in] _visible True for visible, false for invisible.
      public: void SetVisible(bool _visible);

      /// \brief Set the level for this manip.
      /// \param[in] _level The level for this manip.
      public: void SetLevel(const int _level);

      /// \brief Get the level for this manip.
      /// \return The level for this manip.
      public: int GetLevel() const;

      /// \brief Qt signal emitted when the manip's color has changed from the
      /// 3D view.
      /// \param[in] _color New color.
      Q_SIGNALS: void ColorChanged(QColor _color);

      /// \brief Qt callback when the pose of the associated editor item has
      /// changed.
      /// \param[in] _x New X position in pixel coordinates.
      /// \param[in] _y New Y position in pixel coordinates.
      /// \param[in] _z New Z position in pixel coordinates.
      /// \param[in] _roll New roll rotation in degrees.
      /// \param[in] _pitch New pitch rotation in degrees.
      /// \param[in] _yaw New yaw rotation in degrees.
      private slots: void OnPoseChanged(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \brief Qt callback to nofiy the pose of the associated editor item's
      /// origin has changed.
      /// \param[in] _x New X position in pixel coordinates.
      /// \param[in] _y New Y position in pixel coordinates.
      /// \param[in] _z New Z position in pixel coordinates.
      /// \param[in] _roll New roll rotation in degrees.
      /// \param[in] _pitch New pitch rotation in degrees.
      /// \param[in] _yaw New yaw rotation in degrees.
      private slots: void OnPoseOriginTransformed(double _x, double _y,
          double _z, double _roll, double _pitch, double _yaw);

      /// \brief Qt callback when the position of the associated editor item has
      /// changed.
      /// \param[in] _x New X position in pixel coordinates.
      /// \param[in] _y New Y position in pixel coordinates.
      /// \param[in] _z New Z position in pixel coordinates.
      private slots: void OnPositionChanged(double _x, double _y, double _z);

      /// \brief Qt callback when the rotation of the associated editor item has
      /// changed.
      /// \param[in] _roll New roll rotation in degrees.
      /// \param[in] _pitch New pitch rotation in degrees.
      /// \param[in] _yaw New yaw rotation in degrees.
      private slots: void OnRotationChanged(double _roll, double _pitch,
          double _yaw);

      /// \brief Qt callback when the size of the associated editor item has
      /// changed.
      /// \param[in] _width New width in pixels.
      /// \param[in] _depth New depth in pixels.
      /// \param[in] _height New height in pixels.
      private slots: void OnSizeChanged(double _width, double _depth,
          double _height);

      /// \brief Qt callback when the width of the associated editor item has
      /// changed.
      /// \param[in] _width New width in pixels.
      private slots: void OnWidthChanged(double _width);

      /// \brief Qt callback when the height of the associated editor item has
      /// changed.
      /// \param[in] _height New height in pixels.
      private slots: void OnHeightChanged(double _height);

      /// \brief Qt callback when the depth of the associated editor item has
      /// changed
      /// \param[in] _depth New depth in pixels.
      private slots: void OnDepthChanged(double _depth);

      /// \brief Qt callback when the X position of the associated editor item
      /// has changed.
      /// \param[in] _posX New X position in pixel coordinates.
      private slots: void OnPosXChanged(double _posX);

      /// \brief Qt callback when the Y position of the associated editor item
      /// has changed.
      /// \param[in] _posY New Y position in pixel coordinates.
      private slots: void OnPosYChanged(double _posY);

      /// \brief Qt callback when the Z position of the associated editor item
      /// has changed.
      /// \param[in] _posZ New Z position in pixel coordinates.
      private slots: void OnPosZChanged(double _posZ);

      /// \brief Qt callback when the yaw rotation of the associated editor item
      /// has changed.
      /// \param[in] _posZ New yaw rotation in degrees.
      private slots: void OnYawChanged(double _yaw);

      /// \brief Qt callback when the level of the associated editor item
      /// has changed.
      /// \param[in] _level New level.
      private slots: void OnLevelChanged(int _level);

      /// \brief Qt callback when the 3D visual's color has been changed from
      /// the associated editor item.
      /// \param[in] _color New color.
      private slots: void OnColorChanged(QColor _color);

      /// \brief Qt callback when the 3D visual's texture has been changed from
      /// the associated editor item.
      /// \param[in] _texture New texture.
      private slots: void OnTextureChanged(QString _texture);

      /// \brief Qt callback when the 3D visual's transparency has been changed
      /// from the associated editor item.
      /// \param[in] _transparency Transparency.
      private slots: void OnTransparencyChanged(float _transparency);

      /// \brief Qt callback when the associated editor item has been deleted.
      private slots: void OnDeleted();

      /// \brief Callback received when the building level being edited has
      /// changed. Do not confuse with OnLevelChange, where the manip's level
      /// is changed.
      /// \param[in] _level The level that is currently being edited.
      private: void OnChangeLevel(int _level);

      /// \brief Name of the manip.
      private: std::string name;

      /// \brief A pointer to the visual managed by the manip.
      private: rendering::VisualPtr visual;

      /// \brief Size of the manipular.
      private: math::Vector3 size;

      /// \brief Pose of the manip.
      private: math::Pose pose;

      /// \brief Maker that manages this manip.
      private: BuildingMaker *maker;

      /// \brief A list of attached manips.
      private: std::vector<BuildingModelManip *> attachedManips;

      /// \brief Parent manip.
      private: BuildingModelManip *parent;

      /// \brief Visual's transparency.
      private: double transparency;

      /// \brief Visual's color.
      private: common::Color color;

      /// \brief Visual's texture.
      private: std::string texture;

      /// \brief Level this manipulator is on.
      private: int level;

      /// \brief A list of gui editor events connected to this view.
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}
#endif
