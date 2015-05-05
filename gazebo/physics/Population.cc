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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Kmeans.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/physics/Population.hh"
#include "gazebo/physics/PopulationPrivate.hh"
#include "gazebo/physics/World.hh"


using namespace gazebo;
using namespace common;
using namespace physics;

//////////////////////////////////////////////////
Population::Population(sdf::ElementPtr _sdf, boost::shared_ptr<World> _world)
  : dataPtr(new PopulationPrivate)
{
  this->dataPtr->world = _world;
  if (_sdf->HasElement("population"))
    this->dataPtr->populationElem = _sdf->GetElement("population");
}

//////////////////////////////////////////////////
Population::~Population()
{
}

//////////////////////////////////////////////////
bool Population::PopulateAll()
{
  GZ_ASSERT(this->dataPtr->populationElem, "<population> SDF element is NULL");

  sdf::ElementPtr popElem = this->dataPtr->populationElem;
  bool result = true;

  // Iterate through all the population elements in the sdf.
  while (popElem)
  {
    if (!this->PopulateOne(popElem))
      result = false;
    popElem = popElem->GetNextElement("population");
  }

  return result;
}

//////////////////////////////////////////////////
bool Population::PopulateOne(const sdf::ElementPtr _population)
{
  std::vector<math::Vector3> objects;
  PopulationParams params;

  GZ_ASSERT(_population, "'_population' parameter is NULL");

  if (!this->ParseSdf(_population, params))
    return false;

  // Generate the set of poses based on the region and distribution.
  if (params.region == "box" && params.distribution == "random")
    this->CreatePosesBoxRandom(params, objects);
  else if (params.region == "box" && params.distribution == "uniform")
    this->CreatePosesBoxUniform(params, objects);
  else if (params.distribution == "grid")
    this->CreatePosesBoxGrid(params, objects);
  else if (params.region == "box" && params.distribution == "linear-x")
    this->CreatePosesBoxLinearX(params, objects);
  else if (params.region == "box" && params.distribution == "linear-y")
    this->CreatePosesBoxLinearY(params, objects);
  else if (params.region == "box" && params.distribution == "linear-z")
    this->CreatePosesBoxLinearZ(params, objects);
  else if (params.region == "cylinder" && params.distribution == "random")
    this->CreatePosesCylinderRandom(params, objects);
  else if (params.region == "cylinder" && params.distribution == "uniform")
    this->CreatePosesCylinderUniform(params, objects);
  else
  {
    gzerr << "Unrecognized combination of region [" << params.region << "] and "
          << "distribution [" << params.distribution << "]" << std::endl;
    return false;
  }

  // Create an sdf containing the model description.
  sdf::SDF sdf;
  sdf.SetFromString("<sdf version ='1.5'>" + params.modelSdf + "</sdf>");

  for (size_t i = 0; i < objects.size(); ++i)
  {
    math::Vector3 p(objects[i].x, objects[i].y, objects[i].z);

    // Create a unique model for each clone.
    std::string cloneSdf = sdf.ToString();
    std::string delim = "model name='";
    size_t first = cloneSdf.find(delim) + delim.size();
    size_t last = cloneSdf.find("'", first);
    std::string newName = params.modelName + std::string("_clone_") +
      boost::lexical_cast<std::string>(i);
    cloneSdf.replace(first, last - first, newName);

    // Insert the <pose> element.
    std::string endDelim = "'>";
    first = cloneSdf.find(delim) + delim.size();
    last = cloneSdf.find(endDelim, first);
    std::string pose = "\n    <pose>" +
      boost::lexical_cast<std::string>(p.x) + " " +
      boost::lexical_cast<std::string>(p.y) + " " +
      boost::lexical_cast<std::string>(p.z) + " 0 0 0</pose>";
    cloneSdf.insert(last + endDelim.size(), pose);

    this->dataPtr->world->InsertModelString(cloneSdf);
  }

  return true;
}

/////////////////////////////////////////////////
bool Population::ElementFromSdf(const sdf::ElementPtr &_sdfElement,
  const std::string &_element, sdf::ElementPtr &_value)
{
  if (_sdfElement->HasElement(_element))
  {
    _value = _sdfElement->GetElement(_element);
    return true;
  }
  gzerr << "Unable to find <" << _element << "> inside the population tag"
        << std::endl;
  return false;
}

/////////////////////////////////////////////////
bool Population::ParseSdf(sdf::ElementPtr _population,
  PopulationParams &_params)
{
  GZ_ASSERT(_population, "'_population' parameter is NULL");

  // Read the model element.
  sdf::ElementPtr model;
  if (!this->ElementFromSdf(_population, "model", model))
    return false;

  _params.modelSdf = model->ToString("");
  _params.modelName = model->Get<std::string>("name");

  // Read the pose.
  math::Pose pose;
  if (!this->ValueFromSdf(_population, "pose", _params.pose))
    return false;

  // Read the distribution element.
  sdf::ElementPtr distribution;
  if (!this->ElementFromSdf(_population, "distribution", distribution))
    return false;

  // Read the distribution type.
  if (!this->ValueFromSdf<std::string>(distribution, "type",
    _params.distribution))
  {
    return false;
  }

  if ((_params.distribution != "random")   &&
      (_params.distribution != "uniform")  &&
      (_params.distribution != "grid")     &&
      (_params.distribution != "linear-x") &&
      (_params.distribution != "linear-y") &&
      (_params.distribution != "linear-z"))
  {
    gzerr << "Unknown distribution type [" << _params.distribution << "]"
          << std::endl;
    return false;
  }

  // Models evenly distributed in a 2D grid pattern.
  if (_params.distribution == "grid")
  {
    // Read the number of rows.
    if (!this->ValueFromSdf<int>(distribution, "rows", _params.rows))
      return false;

    // Sanity check.
    if (_params.rows <= 0)
    {
      gzwarn << "Incorrect number of rows while populating objects ["
             << _params.rows << "]. Population ignored." << std::endl;
      return false;
    }

    // Read the number of columns.
    if (!this->ValueFromSdf<int>(distribution, "cols", _params.cols))
      return false;

    // Sanity check.
    if (_params.cols <= 0)
    {
      gzwarn << "Incorrect number of columns while populating objects ["
             << _params.cols << "]. Population ignored." << std::endl;
      return false;
    }

    // Read the <step> value used to separate each model in the grid.
    if (!this->ValueFromSdf<math::Vector3>(distribution, "step", _params.step))
      return false;

    // Align the origin of the grid with 'pose'.
    if (_params.cols % 2 == 0)
    {
      _params.pose.pos.x -=
        (_params.step.x * (_params.cols - 2) / 2.0) + (_params.step.x / 2.0);
    }
    else
      _params.pose.pos.x -= _params.step.x * (_params.cols - 1) / 2.0;

    if (_params.rows % 2 == 0)
    {
      _params.pose.pos.y -=
        (_params.step.y * (_params.rows - 2) / 2.0) + (_params.step.y / 2.0);
    }
    else
      _params.pose.pos.y -= _params.step.y * (_params.rows - 1) / 2.0;
  }
  else
  {
    // Read the model_count element.
    if (!this->ValueFromSdf<int>(_population, "model_count",
      _params.modelCount))
    {
      return false;
    }

    // Sanity check.
    if (_params.modelCount <= 0)
    {
      gzwarn << "Trying to populate a non positive number of models ["
             << _params.modelCount << "]. Population ignored." << std::endl;
      return false;
    }

    // Read the region element.
    if (_population->HasElement("box"))
    {
      sdf::ElementPtr box = _population->GetElement("box");
      _params.region = "box";

      // Read the size of the bounding box.
      if (!this->ValueFromSdf<math::Vector3>(box, "size", _params.size))
        return false;

      // Sanity check.
      if (_params.size.x <= 0 || _params.size.y <= 0 || _params.size.z <= 0)
      {
        gzwarn << "Incorrect box size while populating objects ["
               << _params.size << "]. Population ignored." << std::endl;
        return false;
      }

      // Align the origin of the box with 'pose'.
      _params.pose.pos -= _params.size / 2.0;
    }
    else if (_population->HasElement("cylinder"))
    {
      sdf::ElementPtr cylinder = _population->GetElement("cylinder");
      _params.region = "cylinder";

      // Read the radius of the cylinder's base.
      if (!this->ValueFromSdf<double>(cylinder, "radius", _params.radius))
        return false;

      // Sanity check.
      if (_params.radius <= 0)
      {
        gzwarn << "Incorrect radius value while populating objects ["
               << _params.radius << "]. Population ignored." << std::endl;
        return false;
      }

      // Read the cylinder's length.
      if (!this->ValueFromSdf<double>(cylinder, "length", _params.length))
        return false;

      // Sanity check.
      if (_params.length <= 0)
      {
        gzwarn << "Incorrect length value while populating objects ["
               << _params.length << "]. Population ignored." << std::endl;
        return false;
      }
    }
    else
    {
      gzerr << "I have not found a valid region. 'box' or 'cylinder' are"
            << " the valid region types" << std::endl;
      return false;
    }
  }

  return true;
}

/////////////////////////////////////////////////
void Population::CreatePosesBoxRandom(const PopulationParams &_populParams,
  std::vector<math::Vector3> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  _poses.clear();
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    math::Pose offset(math::Rand::GetDblUniform(0, _populParams.size.x),
                      math::Rand::GetDblUniform(0, _populParams.size.y),
                      math::Rand::GetDblUniform(0, _populParams.size.z),
                      0, 0, 0);

    _poses.push_back((offset + _populParams.pose).pos);
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void Population::CreatePosesBoxUniform(const PopulationParams &_populParams,
  std::vector<math::Vector3> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  std::vector<math::Vector3> obs;

  // Step1: Sample points in a box.
  double x = 0.0;
  double y = 0.0;
  while (y < _populParams.size.y)
  {
    while (x < _populParams.size.x)
    {
      math::Vector3 p;
      p.x = x;
      p.y = y;
      p.z = math::Rand::GetDblUniform(0, _populParams.size.z);
      obs.push_back(p);
      x += .1;
    }
    x = 0.0;
    y += .1;
  }

  // Step2: Cluster the sampled points in 'modelCount' clusters.
  std::vector<math::Vector3> centroids;
  std::vector<unsigned int> labels;
  math::Kmeans kmeans(obs);
  kmeans.Cluster(_populParams.modelCount, centroids, labels);

  // Step3: Create the list of object positions.
  _poses.clear();
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    math::Pose p(centroids[i], math::Quaternion(0, 0, 0));
    _poses.push_back((p + _populParams.pose).pos);
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void Population::CreatePosesBoxGrid(const PopulationParams &_populParams,
  std::vector<math::Vector3> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  _poses.clear();
  math::Pose offset = math::Pose::Zero;
  for (int i = 0; i < _populParams.rows; ++i)
  {
    for (int j = 0; j < _populParams.cols; ++j)
    {
      _poses.push_back((offset + _populParams.pose).pos);
      offset.pos.x += _populParams.step.x;
    }
    offset.pos.x = 0;
    offset.pos.y += _populParams.step.y;
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.rows * _populParams.cols ==
    static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void Population::CreatePosesBoxLinearX(const PopulationParams &_populParams,
  std::vector<math::Vector3> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  // Evenly placed in a row along the global x-axis.
  _poses.clear();
  math::Pose offset = math::Pose::Zero;
  offset.pos.y = _populParams.size.y / 2.0;
  offset.pos.z = _populParams.size.z / 2.0;
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    offset.pos.x =
      _populParams.size.x * i / static_cast<double>(_populParams.modelCount);
    _poses.push_back((offset + _populParams.pose).pos);
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void Population::CreatePosesBoxLinearY(const PopulationParams &_populParams,
  std::vector<math::Vector3> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  // Evenly placed in a row along the global y-axis.
  _poses.clear();
  math::Pose offset = math::Pose::Zero;
  offset.pos.x = _populParams.size.x / 2.0;
  offset.pos.z = _populParams.size.z / 2.0;
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    offset.pos.y =
      _populParams.size.y * i / static_cast<double>(_populParams.modelCount);
    _poses.push_back((offset + _populParams.pose).pos);
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void Population::CreatePosesBoxLinearZ(const PopulationParams &_populParams,
  std::vector<math::Vector3> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  // Evenly placed in a row along the global z-axis.
  _poses.clear();
  math::Pose offset = math::Pose::Zero;
  offset.pos.x = _populParams.size.x / 2.0;
  offset.pos.y = _populParams.size.y / 2.0;
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    offset.pos.z =
      _populParams.size.z * i / static_cast<double>(_populParams.modelCount);
    _poses.push_back((offset + _populParams.pose).pos);
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void Population::CreatePosesCylinderRandom(const PopulationParams &_populParams,
  std::vector<math::Vector3> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  _poses.clear();
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    double ang = math::Rand::GetDblUniform(0, 2 * M_PI);
    double r = math::Rand::GetDblUniform(0, _populParams.radius);
    math::Pose offset = math::Pose::Zero;
    offset.pos.x = r * cos(ang);
    offset.pos.y = r * sin(ang);
    offset.pos.z = math::Rand::GetDblUniform(0, _populParams.length);
    _poses.push_back((offset + _populParams.pose).pos);
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}

/////////////////////////////////////////////////
void Population::CreatePosesCylinderUniform(
  const PopulationParams &_populParams, std::vector<math::Vector3> &_poses)
{
  // _poses should be empty.
  GZ_ASSERT(_poses.empty(), "Output parameter '_poses' is not empty");

  std::vector<math::Vector3> obs;

  // Step1: Sample points in the cylinder.
  unsigned int points = 10000;
  for (size_t i = 0; i < points; ++i)
  {
    double ang = math::Rand::GetDblUniform(0, 2 * M_PI);
    double r = math::Rand::GetDblUniform(0, _populParams.radius);
    math::Vector3 p;
    p.x = r * cos(ang);
    p.y = r * sin(ang);
    p.z = math::Rand::GetDblUniform(0, _populParams.length);
    obs.push_back(p);
  }

  // Step2: Cluster the sampled points in 'modelCount' clusters.
  std::vector<math::Vector3> centroids;
  std::vector<unsigned int> labels;
  math::Kmeans kmeans(obs);
  kmeans.Cluster(_populParams.modelCount, centroids, labels);

  // Step3: Create the list of object positions.
  _poses.clear();
  math::Pose offset = math::Pose::Zero;
  for (int i = 0; i < _populParams.modelCount; ++i)
  {
    offset.pos = centroids[i];
    _poses.push_back((offset + _populParams.pose).pos);
  }

  // Check that we have generated the appropriate number of poses.
  GZ_ASSERT(_populParams.modelCount == static_cast<int>(_poses.size()),
    "Unexpected number of objects while generating a population");
}
