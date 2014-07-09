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

#include <vector>
#include "gazebo/common/Console.hh"
#include "gazebo/math/Kmeans.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/math/Vector3.hh"

using namespace gazebo;
using namespace common;
using namespace math;

//////////////////////////////////////////////////
Kmeans::Kmeans(const std::vector<math::Vector3> &_obs, unsigned int _k,
               uint32_t _seed)
  : k(_k),
    seed(_seed)
{
  // Initialize observations.
  this->SetObservations(_obs);

  // Initialize centroids.
  for (size_t i = 0; i < k; ++i)
  {
    // Choose a random observation.
    Vector3 p = this->obs[Rand::GetIntUniform(0, this->obs.size() - 1)];
    this->centroids.push_back(p);
  }
}

//////////////////////////////////////////////////
Kmeans::~Kmeans()
{
}

//////////////////////////////////////////////////
uint32_t Kmeans::GetSeed()
{
  return this->seed;
}

//////////////////////////////////////////////////
void Kmeans::SetSeed(uint32_t _seed)
{
  this->seed = _seed;
}

//////////////////////////////////////////////////
std::vector<math::Vector3> Kmeans::GetObservations()
{
  return this->obs;
}

//////////////////////////////////////////////////
void Kmeans::SetObservations(const std::vector<math::Vector3> &_obs)
{
  this->obs.clear();
  for (size_t i = 0; i < _obs.size(); ++i)
    this->obs.push_back(_obs[i]);
}

//////////////////////////////////////////////////
unsigned int Kmeans::GetNumClusters()
{
  return k;
}

//////////////////////////////////////////////////
void Kmeans::SetNumClusters(unsigned int _k)
{
  this->k = _k;
}

//////////////////////////////////////////////////
bool Kmeans::Cluster(std::vector<Vector3> &_centroids,
                     std::vector<unsigned int> &_labels)
{
  size_t changed = 0;
  // ToDo(caguero): Use gzerr
  // Sanity check
  if (this->obs.size() == 0)
  {
    std::cerr << "The set of observations is empty." << std::endl;
    return false;
  }

  if (this->k == 0)
  {
    std::cerr << "The number of clusters should not be zero." << std::endl;
    return false;
  }

  if (this->k > this->obs.size())
  {
    std::cerr << "The number of clusters has to be lower or equal to the number"
              << " of observations." << std::endl;
    return false;
  }

  // Initialize the size of the vectors;
  this->centroids.resize(this->k);
  this->oldCentroids.resize(this->k);
  this->labels.resize(this->obs.size());
  this->sums.resize(this->k);
  this->counters.resize(this->k);

  // Initialize labels.
  for (size_t i = 0; i < this->obs.size(); ++i)
    this->labels[i] = 0;

  do
  {
    // Reset sums and counters.
    for (size_t i = 0; i < this->centroids.size(); ++i)
    {
      this->sums[i] = math::Vector3(0, 0, 0);
      this->counters[i] = 0;
    }
    changed = 0;

    for (size_t i = 0; i < this->obs.size(); ++i)
    {
      // Update the labels containing the closest centroid for each point.
      unsigned int label = ClosestCentroid(this->obs[i]);
      if (this->labels[i] != label)
      {
        this->labels[i] = label;
        changed++;
      }
      this->sums[label] += this->obs[i];
      this->counters[label]++;
    }

    // Save old centroids.
    this->oldCentroids = centroids;

    // Update the centroids.
    for (size_t i = 0; i < this->centroids.size(); ++i)
      this->centroids[i] = this->sums[i] / this->counters[i];
  }
  while (changed > (this->obs.size() >> 10));

  _centroids = this->centroids;
  _labels = this->labels;
  return true;
}

//////////////////////////////////////////////////
unsigned int Kmeans::ClosestCentroid(Vector3 p)
{
  double min = HUGE_VAL;
  unsigned int minIdx = 0;
  for (size_t i = 0; i < this->centroids.size(); ++i)
  {
    double d = p.Distance(this->centroids[i]);
    if (d < min)
    {
      min = d;
      minIdx = i;
    }
  }
  return minIdx;
}

