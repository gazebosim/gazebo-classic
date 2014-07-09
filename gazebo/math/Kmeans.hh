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
#ifndef _KMEANS_HH_
#define _KMEANS_HH_

#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \class Kmeans Kmeans.hh math/gzmath.hh
    /// \brief K-Means clustering algorithm. Given a set of observations,
    /// k-means partition the observations into k sets so as to minimize the
    /// within-cluster sum of squares.
    /// Description based on http://en.wikipedia.org/wiki/K-means_clustering.
    class GAZEBO_VISIBLE Kmeans
    {
      /// \brief constructor
      /// \param[in] _obs Set of observations to cluster.
      /// \param[in] _k Number of clusters.
      /// \param[in] _seed The seed used to initialize the randon number
      /// generator.
      public: Kmeans(const std::vector<math::Vector3> &_obs, unsigned int _k,
                     uint32_t _seed = 0);

      /// \brief Destructor
      public: virtual ~Kmeans();

      /// \brief Get the seed value.
      /// \return The seed value used to initialize the random number generator.
      public: uint32_t GetSeed();

      /// \brief Set the seed value.
      /// \param[in] _seed The seed used to initialize the randon number
      /// generator.
      public: void SetSeed(uint32_t _seed);

      /// \brief Get the observations to cluster.
      /// \return The vector of observations.
      public: std::vector<math::Vector3> GetObservations();

      /// \brief Set the observations to cluster.
      /// \param[in] _obs The new vector of observations.
      public: void SetObservations(const std::vector<math::Vector3> &_obs);

      /// \brief Get the number of partitions used to cluster.
      /// \return The number of partitions.
      public: unsigned int GetNumClusters();

      /// \brief Set the number of partitions to cluster.
      /// \param[in] _k The number of partitions.
      public: void SetNumClusters(unsigned int _k);

      /// \brief Executes the algorithm.
      /// \param[out] _centroids Vector of centroids.
      /// \return True when the operation succeed or false otherwise.
      public: bool Cluster(std::vector<Vector3> &_centroids,
                           std::vector<unsigned int> &_labels);

      private: unsigned int ClosestCentroid(Vector3 p);

      /// \brief Number of partitions used to cluster.
      private: unsigned int k;

      /// \brief Seed value used to initialize the random number generator.
      private: uint32_t seed;

      /// \brief Observations.
      private: std::vector<Vector3> obs;

      /// \brief Centroids.
      private: std::vector<Vector3> centroids;

      /// \brief Centroids from the previous iteration.
      private: std::vector<Vector3> oldCentroids;

      /// \brief Contains the cluster for each observation.
      private: std::vector<unsigned int> labels;

      private: std::vector<Vector3> sums;

      private: std::vector<unsigned int> counters;
    };
    /// \}
  }
}

#endif



