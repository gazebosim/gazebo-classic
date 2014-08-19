/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_POPULATION_PRIVATE_HH_
#define _GAZEBO_POPULATION_PRIVATE_HH_

#include <string>
#include <sdf/sdf.hh>
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/World.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Stores all the posible parameters that define a population.
    class PopulationParams
    {
      /// \brief The three side lengths of the box.
      public: math::Vector3 size;

      /// \brief Number of rows used when the models are distributed as a grid.
      public: int rows;

      /// \brief Number of columns used when the models are distributed as a
      /// grid.
      public: int cols;

      /// \brief Distance between the models when the objects are distributed
      /// as a grid.
      public: math::Vector3 step;

      /// The reference frame of the population's region.
      public: math::Pose pose;

      /// \brief Radius of the cylinder's base containing the models.
      public: double radius;

      /// \brief Length of the cylinder containing the models.
      public: double length;

      /// \brief Name of the model.
      public: std::string modelName;

      /// \brief Contains the sdf representation of the model.
      public: std::string modelSdf;

      /// \brief Number of models to spawn.
      public: int modelCount;

      /// \brief Object distribution. E.g.: random, grid.
      public: std::string distribution;

      /// \brief Type region in which the objects will be spawned.
      /// E.g.: box, cylinder.
      public: std::string region;
    };

    /// \brief Private data for the Population class
    class PopulationPrivate
    {
      /// \brief The Population's SDF values.
      public: sdf::ElementPtr populationElem;

      /// \brief Pointer to the world.
      public: World *world;
    };
  }
}
#endif
