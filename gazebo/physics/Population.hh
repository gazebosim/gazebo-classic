/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <sdf/sdf.hh>
#include "gazebo/common/Console.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Forward declaration of the private data class.
    class PopulationPrivate;

    /// \brief Stores all the posible parameters that define a population.
    class GAZEBO_VISIBLE PopulationParams
    {
      /// \brief The three side lengths of the box.
      public: math::Vector3 size;

      /// \brief Number of rows used when models are distributed as a grid.
      public: int rows;

      /// \brief Number of columns used when models are distributed as a grid.
      public: int cols;

      /// \brief Distance between models when they are distributed as a grid.
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

      /// \brief Type region in which the objects will be spawned. E.g.: box.
      public: std::string region;
    };

    /// \class Population Population.hh physics/physics.hh
    /// \brief Class that automatically populates an environment with multiple
    /// objects based on several parameters to define the number of objects,
    /// shape of the object distribution or type of distribution.
    class GAZEBO_VISIBLE Population
    {
      /// \brief Constructor. Load an sdf file containing a population element.
      /// \param[in] _sdf SDF parameters.
      /// \param[in] _world Pointer to the world.
      public: Population(sdf::ElementPtr _sdf, boost::shared_ptr<World> _world);

      /// \brief Destructor.
      public: virtual ~Population();

      /// \brief Generate and spawn multiple populations into the world.
      /// \return True when the populations were successfully spawned or false
      /// otherwise.
      public: bool PopulateAll();

      /// \brief Generate and spawn one model population into the world.
      /// \param[in] _population SDF parameter containing the population details
      /// \return True when the population was successfully spawned or false
      /// otherwise.
      private: bool PopulateOne(const sdf::ElementPtr _population);

      /// \brief Read a value from an SDF element. Before reading the value, it
      /// checks if the element exists and print an error message if not found.
      /// \param[in] _sdfElement SDF element containing the value to read.
      /// \param[in] _element SDF label to read. Ex: "model_count", "min".
      /// \param[out] _value Requested value.
      /// \return True if the element was found or false otherwise.
      private: template<typename T> bool ValueFromSdf(
        const sdf::ElementPtr &_sdfElement, const std::string &_element,
        T &_value)
      {
        if (_sdfElement->HasElement(_element))
        {
          _value = _sdfElement->Get<T>(_element);
          return true;
        }
        gzerr << "Unable to find <" << _element << "> inside the population tag"
              << std::endl;
        return false;
      }

      /// \brief Get a requested SDF element from a SDF. Before returning, it
      /// checks if the element exists and print an error message if not found.
      /// \param[in] _sdfElement SDF element containing the requested SDF.
      /// \param[in] _element SDF label to read. Ex: "model", "box".
      /// \param[out] _value Requested SDF element.
      /// \return True if the element was found or false otherwise.
      private: bool ElementFromSdf(const sdf::ElementPtr &_sdfElement,
        const std::string &_element, sdf::ElementPtr &_value);

      /// \brief Parse the sdf file. Some of the output parameters should be
      /// ignored depending on the region's population. For example, if the
      /// region is a box, the parameters '_center', 'radius', and 'height'
      /// should not be used.
      /// \param[in] _population SDF element containing the Population
      /// description.
      /// \param[out] _params Population parameters parsed from the SDF.
      /// \return True when the function succeed or false otherwise.
      private: bool ParseSdf(sdf::ElementPtr _population,
        PopulationParams &_params);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// randomly distributed within a box.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxRandom(const PopulationParams &_populParams,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// uniformly distributed within a box. We use k-means to split the
      /// box in similar subregions.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxUniform(const PopulationParams &_populParams,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses evenly placed in a 2D grid pattern.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _rows Number of rows used when the models are
      /// distributed as a 2D grid.
      /// \param[in] _cols Number of columns used when the models are
      /// distributed as a 2D grid.
      /// \param[in] _step Distance between the models when the objects are
      /// distributed as a 2D grid.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxGrid(const PopulationParams &_populParams,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global x-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxLinearX(const PopulationParams &_populParams,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global y-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxLinearY(const PopulationParams &_populParams,
        std::vector<math::Vector3> &_poses);

      /// \brief Populate a vector of poses with '_modelCount' elements,
      /// evenly placed in a row along the global z-axis.
      /// \param[in] _modelCount Number of poses.
      /// \param[in] _min Minimum corner of the box containing the models.
      /// \param[in] _max Maximum corner of the box containing the models.
      /// \param[out] _poses Vector containing the poses that will be used to
      /// populate models.
      private: void CreatePosesBoxLinearZ(const PopulationParams &_populParams,
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
      private: void CreatePosesCylinderRandom(
        const PopulationParams &_populParams,
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
      private: void CreatePosesCylinderUniform(
        const PopulationParams &_populParams,
        std::vector<math::Vector3> &_poses);

      /// \internal
      /// \brief Pointer to private data.
      private: boost::scoped_ptr<PopulationPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
