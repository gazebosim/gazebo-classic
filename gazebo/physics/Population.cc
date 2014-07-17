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

#include "gazebo/math/Kmeans.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/physics/Population.hh"
#include "gazebo/physics/Model.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Population::Population(sdf::ElementPtr _sdf, World *_world)
  : world(_world)
{
  if (!_sdf->HasElement("population"))
    return;

  this->populationElem = _sdf->GetElement("population");
}

//////////////////////////////////////////////////
Population::~Population()
{
}

//////////////////////////////////////////////////
bool Population::PopulateAll()
{
  sdf::ElementPtr popElem = this->populationElem;
  bool result = true;

  // Iterate trough all the population elements in the sdf.
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
  math::Vector3 min;
  math::Vector3 max;
  math::Vector3 step;
  math::Vector3 center;
  int rows;
  int cols;
  int modelCount;
  double radius;
  double height;
  std::string modelName;
  std::string modelSdf;
  std::string distribution;
  std::string region;
  std::vector<math::Vector3> objects;

  std::cout << "PopulateOne" << std::endl;

  if (!this->ParseSdf(_population, min, max, rows, cols, step, center, radius,
        height, modelName, modelSdf, modelCount, distribution, region))
    return false;

  std::cout << "Parse OK" << std::endl;

  // Generate the population of poses based on the region and distribution.
  if (region == "cuboid" && distribution == "random")
    this->PopulateCuboidRandom(modelCount, min, max, objects);
  else if (region == "cuboid" && distribution == "uniform")
    this->PopulateCuboidUniform(modelCount, min, max, objects);
  else if (region == "cuboid" && distribution == "grid")
    this->PopulateCuboidGrid(min, rows, cols, step, objects);
  else if (region == "cuboid" && distribution == "linear-x")
    this->PopulateCuboidLinearX(modelCount, min, max, objects);
  else if (region == "cuboid" && distribution == "linear-y")
    this->PopulateCuboidLinearY(modelCount, min, max, objects);
  else if (region == "cuboid" && distribution == "linear-z")
    this->PopulateCuboidLinearZ(modelCount, min, max, objects);
  else if (region == "cylinder" && distribution == "random")
    this->PopulateCylinderRandom(modelCount, center, radius, height, objects);
  else if (region == "cylinder" && distribution == "uniform")
    this->PopulateCylinderUniform(modelCount, center, radius, height, objects);
  else
  {
    std::cerr << "Unrecognized combination of region [" << region << "] and "
              << "distribution [" << distribution << "]" << std::endl;
    return false;
  }

  // Create an sdf containing the model description.
  sdf::SDF sdf;
  sdf.SetFromString(
    "<sdf version ='1.5'>" + modelSdf + "</sdf>");

  std::vector<ModelPtr> clonedModels;
  for (size_t i = 0; i < objects.size(); ++i)
  {
    std::cout << "Cloning object " << i << std::endl;
    math::Vector3 p;
    p.x = objects[i].x;
    p.y = objects[i].y;
    p.z = objects[i].z;

    std::string cloneSdf = sdf.ToString();
    std::string delim = "model name='";
    size_t first = cloneSdf.find(delim) + delim.size();
    size_t last = cloneSdf.find("'", first);
    std::string newName = modelName + std::string("_clone_") +
      boost::lexical_cast<std::string>(i);
    cloneSdf.replace(first, last - first, newName);

    sdf::SDF model2Sdf;
    model2Sdf.SetFromString(cloneSdf);
    sdf::ElementPtr modelElem = model2Sdf.root->GetElement("model");
    if (modelElem)
    {
      ModelPtr newModel = this->world->LoadModel(modelElem,
          this->world->rootElement);
      math::Pose newPose(math::Pose(p.x, p.y, p.z, 0, 0, 0));
      newModel->SetWorldPose(newPose);
      newModel->Init();
      newModel->LoadPlugins();
      math::Box box = newModel->GetBoundingBox();
    }
    else
      std::cerr << "ModelElem is NULL" << std::endl;
  }

  return true;
}

/////////////////////////////////////////////////
bool Population::ParseSdf(sdf::ElementPtr _population, math::Vector3 &_min,
  math::Vector3 &_max, int &_rows, int &_cols, math::Vector3 &_step,
  math::Vector3 &_center, double &_radius, double &_height,
  std::string &_modelName, std::string &_modelSdf, int &_modelCount,
  std::string &_distribution, std::string &_region)
{
  // Read all the population elements.
  if (!_population->HasElement("model"))
  {
    std::cerr << "Unable to find the a model inside the population tag."
              << std::endl;
    return false;
  }
  sdf::ElementPtr model = _population->GetElement("model");
  _modelSdf = model->ToString("");
  _modelName = model->Get<std::string>("name");

  if (!_population->HasElement("model_count"))
  {
    std::cerr << "Unable to find <model_count> inside the population tag."
              << std::endl;
    return false;
  }
  _modelCount = _population->Get<int>("model_count");

  if (!_population->HasElement("distribution"))
  {
    std::cerr << "Unable to find <distribution> inside the population tag."
              << std::endl;
    return false;
  }
  _distribution = _population->Get<std::string>("distribution");

  if (!_population->HasElement("region"))
  {
    std::cerr << "Unable to find <region> inside the population tag."
              << std::endl;
    return false;
  }
  sdf::ElementPtr region = _population->GetElement("region");

  if (region->HasElement("cuboid"))
  {
    sdf::ElementPtr cuboid = region->GetElement("cuboid");
    _region = "cuboid";

    if (!cuboid->HasElement("min"))
    {
      std::cerr << "Unable to find <min> inside the cuboid tag." << std::endl;
      return false;
    }
    _min = cuboid->Get<math::Vector3>("min");

    if ((_distribution == "random")   || (_distribution == "uniform")  ||
        (_distribution == "linear-x") || (_distribution == "linear-y") ||
        (_distribution == "linear-z"))
    {
      if (!cuboid->HasElement("max"))
      {
        std::cerr << "Unable to find <max> inside the cuboid tag." << std::endl;
        return false;
      }
      _max = cuboid->Get<math::Vector3>("max");
    }
    else if (_distribution == "grid")
    {
      if (!cuboid->HasElement("rows"))
      {
        std::cerr << "Unable to find <rows> inside the cuboid tag" << std::endl;
        return false;
      }
      _rows = cuboid->Get<int>("rows");

      if (!cuboid->HasElement("cols"))
      {
        std::cerr << "Unable to find <cols> inside the cuboid tag" << std::endl;
        return false;
      }
      _cols = cuboid->Get<int>("cols");

      if (!cuboid->HasElement("step"))
      {
        std::cerr << "Unable to find <step> inside the cuboid tag."
                  << std::endl;
        return false;
      }
      _step = cuboid->Get<math::Vector3>("step");
    }
    else
    {
      std::cerr << "Unknown distribution type [" << _distribution << "]"
                << std::endl;
    }
  }
  else if (region->HasElement("cylinder"))
  {
    sdf::ElementPtr cylinder = region->GetElement("cylinder");
    _region = "cylinder";

    if (!cylinder->HasElement("center"))
    {
      std::cerr << "Unable to find <center> inside the cylinder tag."
                << std::endl;
      return false;
    }
    _center = cylinder->Get<math::Vector3>("center");

    if (!cylinder->HasElement("radius"))
    {
      std::cerr << "Unable to find <radius> inside the cylinder tag."
                << std::endl;
      return false;
    }
    _radius = cylinder->Get<double>("radius");

    if (!cylinder->HasElement("height"))
    {
      std::cerr << "Unable to find <height> inside the cylinder tag."
                << std::endl;
      return false;
    }
    _height = cylinder->Get<double>("height");
  }
  else
  {
    std::cerr << "I have not found a valid region. 'Cuboid' or 'Cylinder' are"
              << " the valid region types" << std::endl;
  }

  return true;
}

/////////////////////////////////////////////////
void Population::PopulateCuboidRandom(int _modelCount,
  const math::Vector3 &_min, const math::Vector3 &_max,
  std::vector<math::Vector3> &_poses)
{
  double dx = fabs(_max.x - _min.x);
  double dy = fabs(_max.y - _min.y);
  double dz = fabs(_max.z - _min.z);

  _poses.clear();
  for (int i = 0; i < _modelCount; ++i)
  {
    math::Vector3 p;
    p.x = std::min(_min.x, _max.x) + math::Rand::GetDblUniform(0, dx);
    p.y = std::min(_min.y, _max.y) + math::Rand::GetDblUniform(0, dy);
    p.z = std::min(_min.z, _max.z) + math::Rand::GetDblUniform(0, dz);

  _poses.push_back(p);
  }
}

/////////////////////////////////////////////////
void Population::PopulateCuboidUniform(int _modelCount,
  const math::Vector3 &_min, const math::Vector3 &_max,
  std::vector<math::Vector3> &_poses)
{
  std::vector<math::Vector3> obs;

  double dx = fabs(_max.x - _min.x);
  double dy = fabs(_max.y - _min.y);
  double dz = fabs(_max.z - _min.z);

  // Step1: Sample points in a cuboid.
  double x = 0.0;
  double y = 0.0;
  while (y < dy)
  {
    while (x < dx)
    {
      math::Vector3 p;
      p.x = x;
      p.y = y;
      p.z = 0;
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
  kmeans.Cluster(_modelCount, centroids, labels);

  // Step3: Create the list of object positions.
  _poses.clear();
  for (int i = 0; i < _modelCount; ++i)
  {
    math::Vector3 p;
    p.x = std::min(_min.x, _max.x) + centroids[i].x;
    p.y = std::min(_min.y, _max.y) + centroids[i].y;
    p.z = std::min(_min.z, _max.z) + math::Rand::GetDblUniform(0, dz);
    _poses.push_back(p);
  }
}

/////////////////////////////////////////////////
void Population::PopulateCuboidGrid(const math::Vector3 &_min, int _rows,
  int _cols, const math::Vector3 &_step, std::vector<math::Vector3> &_poses)
{
  _poses.clear();
  math::Vector3 p = _min;
  for (int i = 0; i < _rows; ++i)
  {
    for (int j = 0; j < _cols; ++j)
    {
      _poses.push_back(p);
      p.x += _step.x;
    }
    p.x = _min.x;
    p.y += _step.y;
  }
}

/////////////////////////////////////////////////
void Population::PopulateCuboidLinearX(int _modelCount,
  const math::Vector3 &_min, const math::Vector3 &_max,
  std::vector<math::Vector3> &_poses)
{
  double dx = fabs(_max.x - _min.x);

  // Evenly placed in a row along the global x-axis.
  _poses.clear();
  for (int i = 0; i < _modelCount; ++i)
  {
    math::Vector3 p;
    p.x = std::min(_min.x, _max.x) + i * dx / static_cast<double>(_modelCount);
    p.y = (_min.y + _max.y) / 2.0;
    p.z = (_min.z + _max.z) / 2.0;
    _poses.push_back(p);
  }
}

/////////////////////////////////////////////////
void Population::PopulateCuboidLinearY(int _modelCount,
  const math::Vector3 &_min, const math::Vector3 &_max,
  std::vector<math::Vector3> &_poses)
{
  double dy = fabs(_max.y - _min.y);

  // Evenly placed in a row along the global x-axis.
  _poses.clear();
  for (int i = 0; i < _modelCount; ++i)
  {
    math::Vector3 p;
    p.x = (_min.x + _max.x) / 2.0;
    p.y = std::min(_min.y, _max.y) + i * dy / static_cast<double>(_modelCount);
    p.z = (_min.z + _max.z) / 2.0;
    _poses.push_back(p);
  }
}

/////////////////////////////////////////////////
void Population::PopulateCuboidLinearZ(int _modelCount,
  const math::Vector3 &_min, const math::Vector3 &_max,
  std::vector<math::Vector3> &_poses)
{
  double dz = fabs(_max.z - _min.z);

  // Evenly placed in a row along the global x-axis.
  _poses.clear();
  for (int i = 0; i < _modelCount; ++i)
  {
    math::Vector3 p;
    p.x = (_min.x + _max.x) / 2.0;
    p.y = (_min.y + _max.y) / 2.0;
    p.z = std::min(_min.z, _max.z) + i * dz / static_cast<double>(_modelCount);
    _poses.push_back(p);
  }
}

/////////////////////////////////////////////////
void Population::PopulateCylinderRandom(int _modelCount,
  const math::Vector3 &_center, double _radius, double _height,
  std::vector<math::Vector3> &_poses)
{
  _poses.clear();
  for (int i = 0; i < _modelCount; ++i)
  {
    double ang = math::Rand::GetDblUniform(0, 2 * M_PI);
    double r = math::Rand::GetDblUniform(0, _radius);
    math::Vector3 p;
    p.x = _center.x + r * cos(ang);
    p.y = _center.y + r * sin(ang);
    p.z = _center.z + math::Rand::GetDblUniform(0, _height);
    _poses.push_back(p);
  }
}

/////////////////////////////////////////////////
void Population::PopulateCylinderUniform(int _modelCount,
  const math::Vector3 &_center, double _radius, double _height,
  std::vector<math::Vector3> &_poses)
{
  std::vector<math::Vector3> obs;

  // Step1: Sample points in the cylinder.
  unsigned int points = 10000;
  for (size_t i = 0; i < points; ++i)
  {
    double ang = math::Rand::GetDblUniform(0, 2 * M_PI);
    double r = math::Rand::GetDblUniform(0, _radius);
    math::Vector3 p;
    p.x = _center.x + r * cos(ang);
    p.y = _center.y + r * sin(ang);
    p.z = _center.z + math::Rand::GetDblUniform(0, _height);
    obs.push_back(p);
  }

  // Step2: Cluster the sampled points in 'modelCount' clusters.
  std::vector<math::Vector3> centroids;
  std::vector<unsigned int> labels;
  math::Kmeans kmeans(obs);
  kmeans.Cluster(_modelCount, centroids, labels);

  // Step3: Create the list of object positions.
  _poses.clear();
  for (int i = 0; i < _modelCount; ++i)
  {
    math::Vector3 p;
    p.x = centroids[i].x;
    p.y = centroids[i].y;
    p.z = math::Rand::GetDblUniform(0, _height);
    _poses.push_back(p);
  }
}
