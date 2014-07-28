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
#ifndef _GAZEBO_POPULATION_HH_
#define _GAZEBO_POPULATION_HH_

#include <string>
#include <vector>
#include <sdf/sdf.hh>
#include "gazebo/physics/World.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Population Population.hh physics/physics.hh
    /// \brief Class that automatically populates an environment with multiple
    /// objects based on several parameters to define the number of objects,
    /// shape of the object distribution or type of distribution.
    class GAZEBO_VISIBLE Population
    {
      /// \brief Constructor. Load an sdf file containing a population element.
      /// \param[in] _sdf SDF parameters.
      /// \param[in] _world Pointer to the world.
      public: Population(sdf::ElementPtr _sdf, World *_world);

      /// \brief Destructor.
      public: virtual ~Population();

      /// \brief Generate and spawn multiple populations into the world.
      /// \return True when the populations were successfully spawned or false
      /// otherwise.
      public: bool PopulateAll();

      /// \brief Generate and spawn one model population into the world.
      /// \brief SDF parameter containing the population details.
      /// \return True when the population was successfully spawned or false
      /// otherwise.
      private: bool PopulateOne(const sdf::ElementPtr _population);

      /// \brief Parse the sdf file. Some of the output parameters should be
      /// ignored depending on the region's population. For example, if the
      /// region is a cuboid, the parameters '_center', 'radius', and 'height'
      /// should not be used.
      /// \param[in] _population SDF element containing the Population
      /// description.
      /// \param[out] _min Minimum corner of the cuboid containing the models.
      /// \param[out] _max Maximum corner of the cuboid containing the models.
      /// \param[out] _rows Number of rows used when the models are
      /// distributed as a 2D grid.
      /// \param[out] _cols Number of columns used when the models are
      /// distributed as a 2D grid.
      /// \param[out] _step Distance between the models when the objects are
      /// distributed as a 2D grid.
      /// \param[out] _center Center of the cylinder's base containing
      /// the models.
      /// \param[out] _radius Radius of the cylinder's base containing
      /// the models.
      /// \param[out] _height Height of the cylinder containing the models.
      /// \param[out] _modelName Name of the model.
      /// \param[out] _modelSdf Contains the sdf representation of the model.
      /// \param[out] _modelCount Number of models to spawn.
      /// \param[out] _distribution Object distribution. E.g.: random, grid.
      /// \param[out] _region Type region in which the objects will be spawned.
      /// E.g.: cuboid, cylinder.
      /// \return True when the function succeed or false otherwise.
      private: bool ParseSdf(sdf::ElementPtr _population, math::Vector3 &_min,
        math::Vector3 &_max, int &_rows, int &_cols, math::Vector3 &_step,
        math::Vector3 &_center, double &_radius, double &_height,
        std::string &_modelName, std::string &_modelSdf, int &_modelCount,
        std::string &_distribution, std::string &_region);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// randomly distributed within a cuboid.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the cuboid containing the models.
      /// \param[in] _max Maximum corner of the cuboid containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void PopulateCuboidRandom(int _modelCount,
        const math::Vector3 &_min, const math::Vector3 &_max,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// uniformly distributed within a cuboid. We use k-means to split the
      /// cuboid in similar subregions.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the cuboid containing the models.
      /// \param[in] _max Maximum corner of the cuboid containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void PopulateCuboidUniform(int _modelCount,
        const math::Vector3 &_min, const math::Vector3 &_max,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses evenly placed in a 2D grid pattern.
      /// \param[in] _min Minimum corner of the cuboid containing the models.
      /// \param[in] _rows Number of rows used when the models are
      /// distributed as a 2D grid.
      /// \param[in] _cols Number of columns used when the models are
      /// distributed as a 2D grid.
      /// \param[in] _step Distance between the models when the objects are
      /// distributed as a 2D grid.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void PopulateCuboidGrid(const math::Vector3 &_min, int _rows,
        int _cols, const math::Vector3 &_step,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global x-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the cuboid containing the models.
      /// \param[in] _max Maximum corner of the cuboid containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void PopulateCuboidLinearX(int _modelCount,
        const math::Vector3 &_min, const math::Vector3 &_max,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global y-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the cuboid containing the models.
      /// \param[in] _max Maximum corner of the cuboid containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void PopulateCuboidLinearY(int _modelCount,
        const math::Vector3 &_min, const math::Vector3 &_max,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global z-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the cuboid containing the models.
      /// \param[in] _max Maximum corner of the cuboid containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void PopulateCuboidLinearZ(int _modelCount,
        const math::Vector3 &_min, const math::Vector3 &_max,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// randomly distributed within a cylinder.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _center Center of the cylinder's base containing
      /// the models.
      /// \param[in] _radius Radius of the cylinder's base containing
      /// the models
      /// \param[in] _height Height of the cylinder containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void PopulateCylinderRandom(int _modelCount,
        const math::Vector3 &_center, double _radius, double _height,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// uniformly distributed within a cylinder. We use k-means to split the
      /// cylinder in similar subregions.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _center Center of the cylinder's base containing
      /// the models.
      /// \param[in] _radius Radius of the cylinder's base containing
      /// the models
      /// \param[in] _height Height of the cylinder containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void PopulateCylinderUniform(int _modelCount,
        const math::Vector3 &_center, double _radius, double _height,
        std::vector<math::Vector3> &_poses);

      /// \brief The Population's SDF values.
      private: sdf::ElementPtr populationElem;

      /// \brief Pointer to the world.
      private: World *world;
    };
    /// \}
  }
}
#endif
