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
#ifndef _BUILDING_MAKER_HH_
#define _BUILDING_MAKER_HH_

#include <list>
#include <string>
#include <vector>
#include <map>
#include "gazebo/sdf/sdf.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gui/EntityMaker.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

  namespace gui
  {
    class EntityMaker;

    class EditorItem;

    class ModelManip;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingMaker BuildingMaker.hh
    /// \brief Create and manage 3D visuals of a building.
    class BuildingMaker : public EntityMaker
    {
      /// \brief Constructor
      public: BuildingMaker();

      /// \brief Destructor
      public: virtual ~BuildingMaker();

      /// \brief Set the name of this model.
      /// \param[in] _topicName Name of the model to set to.
      public: void SetModelName(const std::string &_modelName);

      /// \brief Finish the model and create the entity on the gzserver.
      public: void FinishModel();

      /// \brief Add a building part to the model.
      /// \param[in] _type Type of the building part.
      /// \param[in] _size Size of the building part.
      /// \param[in] _pos Position of the building part in world coordinates.
      /// \param[in] _angle Yaw rotation of the building part in radians.
      /// \return The name of the 3D building part that was added.
      public: std::string AddPart(const std::string &_type,
          const QVector3D &_size, const QVector3D &_pos, double _angle);

      /// \brief Add a wall to the model.
      /// \param[in] _size Size of the wall.
      /// \param[in] _pos Position of the wall in world coordinates.
      /// \param[in] _angle Yaw rotation of the wall in radians.
      /// \return The name of the 3D wall that was added.
      public: std::string AddWall(const QVector3D &_size, const QVector3D &_pos,
          double _angle);

      /// \brief Add a window to the model.
      /// \param[in] _size Size of the window.
      /// \param[in] _pos Position of the window in world coordinates.
      /// \param[in] _angle Yaw rotation of the window in radians.
      /// \return The name of the 3D window that was added
      public: std::string AddWindow(const QVector3D &_size,
          const QVector3D &_pos, double _angle);

      /// \brief Add a door to the model.
      /// \param[in] _size Size of the window.
      /// \param[in] _pos Position of the door in world coordinates.
      /// \param[in] _angle Yaw rotation of the door in radians.
      public: std::string AddDoor(const QVector3D &_size, const QVector3D &_pos,
          double _angle);

      /// \brief Add a staircase to the model.
      /// \param[in] _size Size of the staircase.
      /// \param[in] _pos Position of the staircase in world coordinates.
      /// \param[in] _angle Yaw rotation of the staircase in radians.
      /// \param[in] _steps Number of steps in the staircase.
      /// \return The name of the 3D staircase that was added
      public: std::string AddStairs(const QVector3D &_size,
          const QVector3D &_pos, double _angle, int _steps);

      /// \brief Add a floor to the model.
      /// \param[in] _size Size of the floor.
      /// \param[in] _pos Position of the floor in world coordinates.
      /// \param[in] _angle Yaw rotation of the floor in radians.
      /// \return The name of the 3D floor that was added.
      public: std::string AddFloor(const QVector3D &_size,
          const QVector3D &_pos, double _angle);

      /// \brief Remove a building part from the model.
      /// \param[in] _partName Name of the building part to remove
      public: void RemovePart(const std::string &_partName);

      /// \brief Remove a wall from the model.
      /// \param[in] _partName Name of the wall to remove
      public: void RemoveWall(const std::string &_wallName);

      /// \brief Connect the 2D editor item Qt signals to the 3D visuals
      /// \param[in] _partName Name of the 3D building part to connect to
      /// \param[in] _item 2D Editor item which emits Qt signals
      public: void ConnectItem(const std::string &_partName,
          const EditorItem *_item);

      /// \brief Attach a building part to another, this is currently used for
      /// making holes in walls and floors.
      /// \param[in] _child Child Item to be attached.
      /// \param[in] _child Parent item to attach to.
      public: void AttachManip(const std::string &_child,
          const std::string &_parent);

      /// \brief Detach a building part from another.
      /// \param[in] _child Child Item to be detached.
      /// \param[in] _child Parent item to detach from.
      public: void DetachManip(const std::string &_child,
          const std::string &_parent);

      /// \brief Helper method to convert size from 2D editor coordinate system
      /// to Gazebo coordinate system
      /// \param[in] _size Qt vector data structure.
      /// \return Size in metric units.
      public: static math::Vector3 ConvertSize(const QVector3D &_size);

      /// \brief Helper method to convert size from 2D editor coordinate system
      /// to Gazebo coordinate system
      /// \param[in] _width width in pixels.
      /// \param[in] _width depth in pixels.
      /// \param[in] _height height in pixels.
      /// \return Size in metric units.
      public: static math::Vector3 ConvertSize(double _width, double _depth,
          double _height);

      /// \brief Helper method to convert pose from 2D editor coordinate system
      /// to Gazebo coordinate system
      /// \param[in] _pos Position in pixels.
      /// \param[in] _pos Rotation in pixels.
      /// \return Pose in metric units.
      public: static math::Pose ConvertPose(const QVector3D &_pos,
          const QVector3D &_rot);

      /// \brief Helper method to convert pose from 2D editor coordinate system
      /// to Gazebo coordinate system
      /// \param[in] _x X position in pixels.
      /// \param[in] _y Y position in pixels.
      /// \param[in] _y Z position in pixels.
      /// \param[in] _roll Roll rotation in degrees.
      /// \param[in] _pitch Pitch rotation in degrees.
      /// \param[in] _yaw Yaw rotation in degrees.
      /// \return Pose in metric units.
      public: static math::Pose ConvertPose(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \param[in] _value Convert value from pixels to metric units
      /// \param[in] _value Value in pixels.
      /// \return Value in metric units.
      public: static double Convert(double _value);

      /// \brief Convert angle from 2D editor unit (degrees) to radians
      /// \param[in] _angle Angle in degrees.
      /// \return Angle in radians.
      public: static double ConvertAngle(double _angle);

      /// \brief Save model to SDF format.
      /// \param[in] _savePath Path to save the SDF to.
      public: void SaveToSDF(const std::string &_savePath);

      /// \brief Reset the building maker and the SDF
      public: void Reset();

      // Documentation inherited
      public: virtual void Start(const rendering::UserCameraPtr _camera);

      // Documentation inherited
      public: virtual void Stop();

      /// \brief Generate the SDF from visuals
      public: void GenerateSDF();

      // Documentation inherited
      public: virtual bool IsActive() const;

      // Documentation inherited
      private: virtual void CreateTheEntity();

      /// \brief Internal init function.
      private: bool Init();

      /// \brief Create an empty model.
      private: std::string CreateModel();

      /// \brief Generate SDF with CSG support.
      private: void GenerateSDFWithCSG();

      /// \brief Get a template SDF string of a simple model.
      private: std::string GetTemplateSDFString();

      /// \brief Internal helper function for QPointF comparison
      private: static bool PointCompareY(const QPointF &_a, const QPointF &_b);

      /// \brief Internal helper function for QRectF comparison
      private: static bool RectCompareX(const QRectF &_a, const QRectF &_b);

      /// \brief Internal helper function for QRectF comparison
      private: static bool RectCompareY(const QRectF &_a, const QRectF &_b);

      /// \brief Subdivide A rectangular surface with holes into multiple
      /// rectangles
      /// \param[in] _surface Parent rectangular surface.
      /// \param[in] _holes A list of rectangular holes on the surface.
      /// \param[in] _subdivisions The resulting smaller rectangles representing
      /// the surface with holes.
      private: void SubdivideRectSurface(const QRectF &_surface,
        const std::vector<QRectF> &_holes, std::vector<QRectF> &_subdivisions);

      /// \brief Callback for the save event.
      private: void OnSave();

      /// \brief Callback for the discard event.
      private: void OnDiscard();

      /// \brief Callback for the done event.
      private: void OnDone();

      /// \brief Callback for the exit.
      private: void OnExit();

      /// \brief Conversion the used by the helper Convert functions.
      public: static double conversionScale;

      /// \brief A map of the building part names to the 3D model manip objects.
      private: std::map<std::string, ModelManip *> allItems;

      /// \brief The model in SDF format.
      private: sdf::SDFPtr modelSDF;

      /// \brief A template SDF of a simple model.
      private: sdf::SDFPtr modelTemplateSDF;

      /// \brief Name of the model.
      private: std::string modelName;

      /// \brief The root visual of the model
      private: rendering::VisualPtr modelVisual;

      /// \brief The pose of the model
      private: math::Pose modelPose;

      /// \brief Counter for the number of walls in the model.
      private: int wallCounter;

      /// \brief Counter for the number of windows in the model.
      private: int windowCounter;

      /// \brief Counter for the number of doors in the model.
      private: int doorCounter;

      /// \brief Counter for the number of stairs in the model.
      private: int stairsCounter;

      /// \brief Counter for the number of floors in the model.
      private: int floorCounter;

      /// \brief Indicate whether the model has been saved before or not.
      private: bool saved;

      /// \brief Path to where the model is saved.
      private: std::string saveLocation;

      /// \brief A list of gui editor events.
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}
#endif
