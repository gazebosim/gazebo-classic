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
#ifndef _MODEL_MANIP_HH_
#define _MODEL_MANIP_HH_

#include <string>
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/rendering/Visual.hh"

namespace gazebo
{
  namespace gui
  {
    class BuildingMaker;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelManip ModelManip.hh
    /// \brief Manipulate a 3D visual associated to a 2D editor item.
    class ModelManip : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: ModelManip();

      /// \brief Destructor
      public: virtual ~ModelManip();

      /// \brief Get the name of the manipulator object.
      /// \return Name of the manipulator object.
      public: std::string GetName() const;

      /// \brief Get the visual this manipulator manages.
      /// \return A pointer to the visual object.
      public: rendering::VisualPtr GetVisual() const;

      /// \brief Set the name of the manipulator object.
      /// \param[in] _name Name to set the manipulator to.
      public: void SetName(const std::string &_name);

      /// \brief Set the visual this manipulator manages.
      /// \param[in] _visual A pointer of the visual object.
      public: void SetVisual(const rendering::VisualPtr &_visual);

      /// \brief Set the maker that the manipulator is managed by.
      /// \param[in] _maker Maker that manages the manipulator.
      public: void SetMaker(BuildingMaker *_maker);

      /// \brief Get the parent of this manipulator.
      /// \return Parent manipulator.
      public: ModelManip *GetParent() const;

      /// \brief Attach a manipulator as a child to this manipulator.
      /// \param[in] _manip Manipulator to be attached.
      public: void AttachManip(ModelManip *_manip);

      /// \brief Detach a child manipulator from this manipulator.
      /// \param[in] _manip Manipulator to be detached.
      public: void DetachManip(ModelManip *_manip);

      /// \brief Set the parent manipulator of this manipulator.
      /// \param[in] _parent Parent manipulator
      public: void SetAttachedTo(ModelManip *_parent);

      /// \brief Detach this manipulator from its parent.
      public: void DetachFromParent();

      /// \brief Get a child manipulator by index.
      /// \param[in] _index Index of the child manipulator.
      public: ModelManip *GetAttachedManip(unsigned int _index) const;

      /// \brief Get the number of child manipulators attached to this
      /// manipulator.
      /// \param[in] _index Index of the child manipulator.
      public: unsigned int GetAttachedManipCount() const;

      /// \brief Get whether or not this manipulator is attached to another.
      /// \return True if attached, false otherwise.
      public: bool IsAttached() const;

      /// \brief Set the pose of the manipulator.
      /// \param[in] _x X position in pixel coordinates.
      /// \param[in] _y Y position in pixel coordinates.
      /// \param[in] _z Z position in pixel coordinates.
      /// \param[in] _roll Roll rotation in degrees.
      /// \param[in] _pitch Pitch rotation in degrees.
      /// \param[in] _yaw Yaw rotation in degrees.
      public: void SetPose(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \brief Set the position of the manipulator.
      /// \param[in] _x X position in pixel coordinates.
      /// \param[in] _y Y position in pixel coordinates.
      /// \param[in] _z Z position in pixel coordinates.
      public: void SetPosition(double _x, double _y, double _z);

      /// \brief Set the rotation of the manipulator.
      /// \param[in] _roll Roll rotation in degrees.
      /// \param[in] _pitch Pitch rotation in degrees.
      /// \param[in] _yaw Yaw rotation in degrees.
      public: void SetRotation(double _roll, double _pitch, double _yaw);

      /// \brief Set the size of the manipulator.
      /// \param[in] _width Width in pixels.
      /// \param[in] _depth Depth in pixels.
      /// \param[in] _height Height pixels.
      public: void SetSize(double _width, double _depth, double _height);

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

      /// \brief Qt callback when the associated editor item has been deleted.
      private slots: void OnDeleted();

      /// \brief Name of the manipulator.
      private: std::string name;

      /// \brief A pointer to the visual managed by the manipulator.
      private: rendering::VisualPtr visual;

      /// \brief Size of the manipular.
      private: math::Vector3 size;

      /// \brief Pose of the manipulator.
      private: math::Pose pose;

      /// \brief Maker that manages this manipulator.
      private: BuildingMaker *maker;

      /// \brief A list of attached manipulators.
      private: std::vector<ModelManip *> attachedManips;

      /// \brief Parent manipulator.
      private: ModelManip * parent;
    };
    /// \}
  }
}
#endif
