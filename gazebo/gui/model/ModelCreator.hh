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
#ifndef _MODEL_CREATOR_HH_
#define _MODEL_CREATOR_HH_

#include <list>
#include <string>
#include <vector>
#include <map>
#include "gazebo/sdf/sdf.hh"
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

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelCreator ModelCreator.hh
    /// \brief Create and manage 3D visuals of a building.
    class ModelCreator : public EntityMaker
    {
      /// \brief Constructor
      public: ModelCreator();

      /// \brief Destructor
      public: virtual ~ModelCreator();

      /// \brief Set the name of this building model.
      /// \param[in] _modelName Name of the model to set to.
      public: void SetModelName(const std::string &_modelName);

      /// \brief Finish the model and create the entity on the gzserver.
       public: void FinishModel();

      /// \brief Add a box to the model.
      /// \param[in] _size Size of the building part.
      /// \param[in] _pose Pose of the box.
      /// \return Name of the box that has been added.
      public: std::string AddBox(
          const math::Vector3 &_size = math::Vector3::One,
          const math::Pose &_pose = math::Pose::Zero);

      /// \brief Remove a part from the model.
      /// \param[in] _partName Name of the part to remove
      public: void RemovePart(const std::string &_partName);

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

      // Documentation inherited
      private: virtual void CreateTheEntity();

      /// \brief Internal init function.
      private: bool Init();

      /// \brief Create an empty model.
      /// \return Name of the model created.
      private: std::string CreateModel();

      /// \brief Get a template SDF string of a simple model.
      private: std::string GetTemplateSDFString();

/*      /// \brief Callback for saving the model.
      private: void OnSave();

      /// \brief Callback for discarding the model.
      private: void OnDiscard();

      /// \brief Callback when the model is to be finished and uploaded on to
      /// the server.
      private: void OnDone();

      /// \brief Callback received when exiting the editor mode.
      private: void OnExit();*/

      /// \brief The building model in SDF format.
      private: sdf::SDFPtr modelSDF;

      /// \brief A template SDF of a simple box model.
      private: sdf::SDFPtr modelTemplateSDF;

      /// \brief Name of the building model.
      private: std::string modelName;

      /// \brief The root visual of the model.
      private: rendering::VisualPtr modelVisual;

      /// \brief The pose of the building model.
      private: math::Pose modelPose;

      /// \brief Indicate whether the model has been saved before or not.
      private: bool saved;

      /// \brief Path to where the model is saved.
      private: std::string saveLocation;

      /// \brief A list of gui editor events connected to the model creator.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Counter for the number of boxes in the model.
      private: int boxCounter;

      /// \brief Counter for the number of cylinders in the model.
      private: int cylinderCounter;

      /// \brief Counter for the number of spheres in the model.
      private: int sphereCounter;

      /// \brief A map of model part names to and their visuals.
      private: std::map<std::string, rendering::VisualPtr> allParts;
    };
    /// \}
  }
}
#endif
