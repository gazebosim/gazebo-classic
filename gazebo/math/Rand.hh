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
/* Desc: Random number generator
 * Author: Nate Koenig
 * Date: 27 May 2009
 */

#ifndef _RAND_HH_
#define _RAND_HH_

#include <boost/random.hpp>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace math
  {
    /// \def GeneratorType
    /// \brief boost::mt19937
    typedef boost::mt19937 GeneratorType;
    /// \def UniformRealDist
    /// \brief boost::uniform_real<double>
    typedef boost::uniform_real<double> UniformRealDist;
    /// \def NormalRealDist
    /// \brief boost::normal_distribution<double>
    typedef boost::normal_distribution<double> NormalRealDist;
    /// \def UniformIntDist
    /// \brief boost::uniform_int<int>
    typedef boost::uniform_int<int> UniformIntDist;

    /// \def URealGen
    /// \brief boost::variate_generator with GeneratorType and UniformRealDist
    typedef boost::variate_generator<GeneratorType&, UniformRealDist > URealGen;
    /// \def NRealGen
    /// \brief boost::variate_generator with GeneratorType and NormalRealDist
    typedef boost::variate_generator<GeneratorType&, NormalRealDist > NRealGen;
    /// \def UIntGen
    /// \brief boost::variate_generator with GeneratorType and UniformIntDist
    typedef boost::variate_generator<GeneratorType&, UniformIntDist > UIntGen;

    /// \addtogroup gazebo_math
    /// \{

    /// \class Rand Rand.hh gzmath/gzmath.hh
    /// \brief Random number generator class
    class GAZEBO_VISIBLE Rand
    {
      /// \brief Set the seed value.
      /// \param[in] _seed The seed used to initialize the randon number
      /// generator.
      public: static void SetSeed(uint32_t _seed);

      /// \brief Get the seed value.
      /// \return The seed value used to initialize the random number
      /// generator.
      public: static uint32_t GetSeed();

      /// \brief Get a double from a uniform distribution
      /// \param[in] _min Minimum bound for the random number
      /// \param[in] _max Maximum bound for the random number
      public: static double GetDblUniform(double _min = 0, double _max = 1);

      /// \brief Get a double from a normal distribution
      /// \param[in] _mean Mean value for the distribution
      /// \param[in] _sigma Sigma value for the distribution
      public: static double GetDblNormal(double _mean = 0, double _sigma = 1);

      /// \brief Get a integer from a uniform distribution
      /// \param[in] _min Minimum bound for the random number
      /// \param[in] _max Maximum bound for the random number
      public: static int GetIntUniform(int _min, int _max);

      /// \brief Get a double from a normal distribution
      /// \param[in] _mean Mean value for the distribution
      /// \param[in] _sigma Sigma value for the distribution
      public: static int GetIntNormal(int _mean, int _sigma);

      // The random number generator
      private: static GeneratorType *randGenerator;

      private: static uint32_t seed;
    };
    /// \}
  }
}
#endif

