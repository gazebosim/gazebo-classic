/*
 * Copyright 2011 Nate Koenig
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

#ifndef RAND_HH
#define RAND_HH

#include <boost/random.hpp>

namespace gazebo
{
  namespace math
  {
    typedef boost::mt19937 GeneratorType;
    typedef boost::uniform_real<double> UniformRealDist;
    typedef boost::normal_distribution<double> NormalRealDist;
    typedef boost::uniform_int<int> UniformIntDist;

    typedef boost::variate_generator<GeneratorType&, UniformRealDist > URealGen;
    typedef boost::variate_generator<GeneratorType&, NormalRealDist > NRealGen;
    typedef boost::variate_generator<GeneratorType&, UniformIntDist > UIntGen;

    /// \addtogroup gazebo_math
    /// \{

    /// \brief Random number generator class
    class Rand
    {
      /// \brief Get a double from a uniform distribution
      /// \param min Minimum bound for the random number
      /// \param max Maximum bound for the random number
      public: static double GetDblUniform(double _min = 0, double _max = 1);

      /// \brief Get a double from a normal distribution
      /// \param mean Mean value for the distribution
      /// \param sigma Sigma value for the distribution
      public: static double GetDblNormal(double _mean = 0, double _sigma = 1);

      /// \brief Get a integer from a uniform distribution
      /// \param min Minimum bound for the random number
      /// \param max Maximum bound for the random number
      public: static int GetIntUniform(int _min, int _max);

      /// \brief Get a double from a normal distribution
      /// \param mean Mean value for the distribution
      /// \param sigma Sigma value for the distribution
      public: static int GetIntNormal(int _mean, int _sigma);

      // The random number generator
      private: static GeneratorType *randGenerator;
    };
    /// \}
  }
}
#endif

