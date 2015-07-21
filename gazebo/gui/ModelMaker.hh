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
#ifndef _GAZEBO_MODEL_MAKER_HH_
#define _GAZEBO_MODEL_MAKER_HH_

#include <list>
#include <string>
#include <sdf/sdf.hh>

#include "gazebo/gui/EntityMaker.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Used to insert new models into the scene.
    class GAZEBO_VISIBLE ModelMaker : public EntityMaker
    {
      /// \enum SimpleShapes
      /// \brief Unique identifiers for each simple shape supported.
      public: enum SimpleShapes {
                  /// \brief Box
                  BOX,
                  /// \brief Sphere
                  SPHERE,
                  /// \brief Cylinder
                  CYLINDER
                };

      /// \brief Constructor
      public: ModelMaker();

      // Documentation inherited
      public: virtual void Stop();

      /// \brief Initialize the model maker with an existing model
      /// \param[in] _modelName Name of existing model in the scene.
      /// \return True if initialization is successful.
      public: bool InitFromModel(const std::string &_modelName);

      /// \brief Initialize the model maker from an SDF string.
      /// \param[in] _data String containing SDF for a model.
      /// \return True if initialization is successful.
      public: bool InitFromSDFString(const std::string &_data);

      /// \brief Initialize the model maker from a file.
      /// \param[in] _filename Path to the file.
      /// \return True if initialization is successful.
      public: bool InitFromFile(const std::string &_filename);

      /// \brief Initialize the model maker to make one of the supported simple
      /// shapes.
      /// \param[in] _shape The desired shape.
      /// \return True if initialization is successful.
      public: bool InitSimpleShape(SimpleShapes _shape);

      // Documentation inherited
      protected: virtual ignition::math::Vector3d EntityPosition() const;

      // Documentation inherited
      protected: virtual void SetEntityPosition(
          const ignition::math::Vector3d &_pos);

      /// \brief Internal init function.
      private: bool Init();

      /// \brief Publish a factory message to create the entity.
      private: virtual void CreateTheEntity();

      /// \brief The model visual being created.
      private: rendering::VisualPtr modelVisual;

      /// \brief The SDF representation of the model.
      private: sdf::SDFPtr modelSDF;

      /// \brief True if the model is being created as a clone of an existing
      /// model.
      private: bool clone;
    };
  }
}
#endif
