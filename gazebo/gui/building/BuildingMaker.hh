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
#ifndef _BUILDING_MAKER_HH_
#define _BUILDING_MAKER_HH_

#include <list>
#include <string>
#include <vector>
#include <map>
#include <sdf/sdf.hh>

#include "gazebo/math/Pose.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gui/EntityMaker.hh"
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

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
    class BuildingModelManip;
    class FinishBuildingDialog;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingMaker BuildingMaker.hh
    /// \brief Create and manage 3D visuals of a building.
    class GAZEBO_VISIBLE BuildingMaker : public EntityMaker
    {
      /// \enum SaveState
      /// \brief Save states for the building editor.
      // NEVER_SAVED: The building has never been saved.
      // ALL_SAVED: All changes have been saved.
      // UNSAVED_CHANGES: Has been saved before, but has unsaved changes.
      private: enum SaveState { NEVER_SAVED, ALL_SAVED, UNSAVED_CHANGES };

      /// \brief Constructor
      public: BuildingMaker();

      /// \brief Destructor
      public: virtual ~BuildingMaker();

      /// \brief Set the name of this building model.
      /// \param[in] _modelName Name of the model to set to.
      public: void SetModelName(const std::string &_modelName);

      /// \brief Finish the model and create the entity on the gzserver.
      public: void FinishModel();

      /// \brief Add a building part to the model.
      /// \param[in] _type Type of the building part.
      /// \param[in] _size Size of the building part.
      /// \param[in] _pos Position of the building part in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the building part in degrees.
      /// \return Name of the 3D building part that has been added.
      public: std::string AddPart(const std::string &_type,
          const QVector3D &_size, const QVector3D &_pos, double _angle);

      /// \brief Add a wall to the model.
      /// \param[in] _size Size of the wall.
      /// \param[in] _pos Position of the wall in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the wall in degrees.
      /// \return Name of the 3D wall that has been added.
      public: std::string AddWall(const QVector3D &_size, const QVector3D &_pos,
          double _angle);

      /// \brief Add a window to the model.
      /// \param[in] _size Size of the window.
      /// \param[in] _pos Position of the window in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the window in degrees.
      /// \return Name of the 3D window that has been added.
      public: std::string AddWindow(const QVector3D &_size,
          const QVector3D &_pos, double _angle);

      /// \brief Add a door to the model.
      /// \param[in] _size Size of the door.
      /// \param[in] _pos Position of the door in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the door in degrees.
      /// \return Name of the 3D door that has been added.
      public: std::string AddDoor(const QVector3D &_size, const QVector3D &_pos,
          double _angle);

      /// \brief Add a staircase to the model.
      /// \param[in] _size Size of the staircase.
      /// \param[in] _pos Position of the staircase in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the staircase in degrees.
      /// \param[in] _steps Number of steps in the staircase.
      /// \return Name of the 3D staircase that has been added.
      public: std::string AddStairs(const QVector3D &_size,
          const QVector3D &_pos, double _angle, int _steps);

      /// \brief Add a floor to the model.
      /// \param[in] _size Size of the floor.
      /// \param[in] _pos Position of the floor in pixel coordinates.
      /// \param[in] _angle Yaw rotation of the floor in radians.
      /// \return Name of the 3D floor that has been added.
      public: std::string AddFloor(const QVector3D &_size,
          const QVector3D &_pos, double _angle);

      /// \brief Remove a building part from the model.
      /// \param[in] _partName Name of the building part to remove
      public: void RemovePart(const std::string &_partName);

      /// \brief Remove a wall from the model.
      /// \param[in] _partName Name of the wall to remove
      public: void RemoveWall(const std::string &_wallName);

      /// \brief Connect the 2D editor item Qt signals to the 3D building part.
      /// \param[in] _partName Name of the 3D building part
      /// \param[in] _item 2D editor item.
      public: void ConnectItem(const std::string &_partName,
          const EditorItem *_item);

      /// \brief Attach a building part to another, this is currently used for
      /// making holes in walls and floors.
      /// \param[in] _child Name of the child building part
      /// \param[in] _parent Name of the parent building part.
      public: void AttachManip(const std::string &_child,
          const std::string &_parent);

      /// \brief Detach a child building part from its parent.
      /// \param[in] _child Name of the child building part.
      /// \param[in] _parent Name of the parent building part.
      public: void DetachManip(const std::string &_child,
          const std::string &_parent);

      /// \brief Detach all child building parts from the given manip.
      /// \param[in] _manip Name of the building part.
      public: void DetachAllChildren(const std::string &_manip);

      /// \brief Helper method to convert size from editor coordinate system
      /// to Gazebo coordinate system.
      /// \param[in] _size Size vector in pixels.
      /// \return Size in metric units.
      public: static math::Vector3 ConvertSize(const QVector3D &_size);

      /// \brief Helper method to convert size from editor coordinate system
      /// to Gazebo coordinate system.
      /// \param[in] _width Width in pixels.
      /// \param[in] _depth Depth in pixels.
      /// \param[in] _height Height in pixels.
      /// \return Size in metric units.
      public: static math::Vector3 ConvertSize(double _width, double _depth,
          double _height);

      /// \brief Helper method to convert pose from editor coordinate system
      /// to Gazebo coordinate system.
      /// \param[in] _pos Position in pixels.
      /// \param[in] _rot Rotation in degrees.
      /// \return Pose with position in metric units and rotation in radians.
      public: static math::Pose ConvertPose(const QVector3D &_pos,
          const QVector3D &_rot);

      /// \brief Helper method to convert pose from editor coordinate system
      /// to Gazebo coordinate system.
      /// \param[in] _x X position in pixels.
      /// \param[in] _y Y position in pixels.
      /// \param[in] _y Z position in pixels.
      /// \param[in] _roll Roll rotation in degrees.
      /// \param[in] _pitch Pitch rotation in degrees.
      /// \param[in] _yaw Yaw rotation in degrees.
      /// \return Pose with position in metric units and rotation in radians.
      public: static math::Pose ConvertPose(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \param[in] _value Convert a value from pixels to metric units
      /// \param[in] _value Value in pixels.
      /// \return Value in metric units.
      public: static double Convert(double _value);

      /// \brief Convert an angle from editor unit to Gazebo unit
      /// \param[in] _angle Angle in degrees.
      /// \return Angle in radians.
      public: static double ConvertAngle(double _angle);

      /// \brief Save model to SDF format.
      /// \param[in] _savePath Path to save the SDF to.
      public: void SaveToSDF(const std::string &_savePath);

      /// \brief Reset the building maker and the SDF.
      public: void Reset();

      // Documentation inherited
      public: virtual void Start(const rendering::UserCameraPtr _camera);

      // Documentation inherited
      public: virtual void Stop();

      /// \brief Generate the SDF from building part visuals.
      public: void GenerateSDF();

      // Documentation inherited
      public: virtual bool IsActive() const;

      /// TODO
      public: void BuildingChanged();

      // Documentation inherited
      private: virtual void CreateTheEntity();

      /// \brief Internal init function.
      private: bool Init();

      /// \brief Create an empty model.
      /// \return Name of the model created.
      private: std::string CreateModel();

      /// \brief Generate SDF with CSG support (to be supported).
      private: void GenerateSDFWithCSG();

      /// \brief Get a template SDF string of a simple model.
      private: std::string GetTemplateSDFString();

      /// \brief Get a template config file for a simple model.
      private: std::string GetTemplateConfigString();

      /// \brief Internal helper function for QPointF comparison used by the
      /// surface subsivision algorithm.
      private: static bool PointCompareY(const QPointF &_a, const QPointF &_b);

      /// \brief Internal helper function for QRectF comparison used by the
      /// surface subsivision algorithm.
      private: static bool RectCompareX(const QRectF &_a, const QRectF &_b);

      /// \brief Internal helper function for QRectF comparison used by the
      /// surface subsivision algorithm.
      private: static bool RectCompareY(const QRectF &_a, const QRectF &_b);

      /// \brief Subdivide a rectangular surface with holes into multiple
      /// smaller rectangles.
      /// \param[in] _surface Parent rectangular surface.
      /// \param[in] _holes A list of rectangular holes on the surface.
      /// \param[in] _subdivisions The resulting smaller rectangles representing
      /// the surface with holes.
      private: void SubdivideRectSurface(const QRectF &_surface,
        const std::vector<QRectF> &_holes, std::vector<QRectF> &_subdivisions);

      /// \brief Helper function to manage writing files to disk.
      private: void SaveModelFiles();

      /// \brief Callback for saving the model.
      /// \param[in] _saveName Name to save the model.
      /// \return True if the user chose to save, false if the user cancelled.
      private: bool OnSave(const std::string &_saveName = "");

      /// \brief Callback for selecting a folder and saving the model.
      /// \param[in] _saveName Name to save the model.
      /// \return True if the user chose to save, false if the user cancelled.
      private: bool OnSaveAs(const std::string &_saveName);

      /// \brief Callback for when the name is changed through the Palette.
      /// \param[in] _modelName The newly entered building name.
      private: void OnNameChanged(const std::string &_modelName);

      /// \brief Callback for newing the model.
      private: void OnNew();

      /// \brief Callback received when exiting the editor mode.
      private: void OnExit();

      /// \brief Callback received when a level on a building model is to
      /// be changed.
      /// \param[in] _level The level that is currently being edited.
      private: void OnChangeLevel(int _level);

      /// \brief Conversion scale used by the Convert helper functions.
      public: static double conversionScale;

      /// \brief A map of building part names to model manip objects which
      /// manage the visuals representing the building part.
      private: std::map<std::string, BuildingModelManip *> allItems;

      /// \brief The building model in SDF format.
      private: sdf::SDFPtr modelSDF;

      /// \brief A template SDF of a simple box model.
      private: sdf::SDFPtr modelTemplateSDF;

      /// \brief Name of the building model.
      private: std::string modelName;

      /// \brief The root visual of the building model.
      private: rendering::VisualPtr modelVisual;

      /// \brief The pose of the building model.
      private: math::Pose modelPose;

      /// \brief Counter for the number of walls in the model.
      private: int wallCounter;

      /// \brief Counter for the number of windows in the model.
      private: int windowCounter;

      /// \brief Counter for the number of doors in the model.
      private: int doorCounter;

      /// \brief Counter for the number of staircases in the model.
      private: int stairsCounter;

      /// \brief Counter for the number of floors in the model.
      private: int floorCounter;

      /// \brief Store the current save state of the model.
      private: enum SaveState currentSaveState;

      /// \brief Default directory to save models: ~/building_editor_models
      private: std::string defaultPath;

      /// \brief Path to where the model is saved.
      private: std::string saveLocation;

      /// \brief Name of the building model's author.
      private: std::string authorName;

      /// \brief Name of the building model's author's email.
      private: std::string authorEmail;

      /// \brief Model description.
      private: std::string description;

      /// \brief Model version.
      private: std::string version;

      /// \brief A list of gui editor events connected to the building maker.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Default name of building model
      private: std::string buildingDefaultName;

      /// \brief A dialog for setting building model name and save location.
      private: FinishBuildingDialog *saveDialog;

      /// \brief The current level that is being edited.
      private: int currentLevel;
    };
    /// \}
  }
}
#endif
