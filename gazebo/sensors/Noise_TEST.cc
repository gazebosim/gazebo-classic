/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include "test/ServerFixture.hh"
#include "gazebo/sensors/Noise.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////
// Helper function that constructs sdf strings for noise element
sdf::ElementPtr NoiseSdf(const std::string &_type, double _mean,
  double _stddev, double _biasMean, double _biasStddev, double _precision)
{
  std::ostringstream noiseStream;
  noiseStream << "<sdf version='1.4'>"
              << "  <noise type='" << _type << "'>"
              << "    <mean>" << _mean << "</mean>"
              << "    <stddev>" << _stddev << "</stddev>"
              << "    <bias_mean>" << _biasMean << "</bias_mean>"
              << "    <bias_stddev>" << _biasStddev << "</bias_stddev>"
              << "    <precision>" << _precision << "</precision>"
              << "  </noise>"
              << "</sdf>";

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("noise.sdf", sdf);
  sdf::readString(noiseStream.str(), sdf);

  return sdf;
}

//////////////////////////////////////////////////
// Test constructor
TEST(NoiseTest, Constructor)
{
  // Construct and nothing else
  {
    sensors::Noise noise;
  }

  // Construct and initialize
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("none", 0, 0, 0, 0, 0));
  }
}

//////////////////////////////////////////////////
// Test noise types
TEST(NoiseTest, Types)
{
  // NONE type
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("none", 0, 0, 0, 0, 0));
  }

  // GAUSSIAN type
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian", 0, 0, 0, 0, 0));
  }

  // GAUSSIAN_QUANTIZED type
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian_quantized", 0, 0, 0, 0, 0));
  }
}

//////////////////////////////////////////////////
// Helper function for testing no noise
void NoNoise(const sensors::Noise &_noise, unsigned int _count)
{
  // Expect no change in input value
  for (unsigned int i = 0; i < _count; ++i)
  {
    double x = math::Rand::GetDblUniform(-1e6, 1e6);
    EXPECT_NEAR(x, _noise.Apply(x), 1e-6);
  }
}

//////////////////////////////////////////////////
// Helper function for testing Gaussian noise
void GaussianNoise(const sensors::Noise &_noise, unsigned int _count)
{
  // Expect no change in input value
  for (unsigned int i = 0; i < _count; ++i)
  {
    double x = math::Rand::GetDblUniform(-1e6, 1e6);
    EXPECT_NEAR(x, _noise.Apply(x), 1e-6);
  }
}

//////////////////////////////////////////////////
// Test noise application
TEST(NoiseTest, Apply)
{
  unsigned int applyCount = 100;

  // NONE type
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("none", 0, 0, 0, 0, 0));

    NoNoise(noise, applyCount);
  }

  // GAUSSIAN type
  {
    double mean, stddev, biasMean, biasStddev;

    // GAUSSIAN with zero means and standard deviations
    // should be the same as NONE
    mean = 0.0;
    stddev = 0.0;
    biasMean = 0.0;
    biasStddev = 0.0;
    {
      sensors::Noise noise;
      noise.Load(NoiseSdf("gaussian", mean, stddev, biasMean, biasStddev, 0));

      NoNoise(noise, applyCount);
    }
 
    // GAUSSIAN with non-zero means and standard deviations, but no bias
    mean = 10.0;
    stddev = 5.0;
    biasMean = 0.0;
    biasStddev = 0.0;
    {
      sensors::Noise noise;
      noise.Load(NoiseSdf("gaussian", mean, stddev, biasMean, biasStddev, 0));

      // Use constant input and repeatedly add noise to it.
      double x = 42.0;
      // boost code from http://stackoverflow.com/questions/3534335
      boost::accumulators::accumulator_set<double,
        boost::accumulators::stats<boost::accumulators::tag::mean,
                                   boost::accumulators::tag::variance > > acc;
      for (unsigned int i = 0; i < applyCount; ++i)
      {
        double y = noise.Apply(x);
        acc(y);
      }

      // The sample mean should be near x+mean, with standard deviation of
      // stddev / sqrt(applyCount)
      // https://onlinecourses.science.psu.edu/stat414/node/167
      // We will use 5 sigma (4e-5 chance of failure)
      double sampleStdDev = 5*stddev / sqrt(applyCount);
      EXPECT_NEAR(boost::accumulators::mean(acc), x+mean, sampleStdDev);

      // The sample variance has the following variance:
      // 2 stddev^4 / (applyCount - 1)
      // en.wikipedia.org/wiki/Variance#Distribution_of_the_sample_variance
      // Again use 5 sigma
      double variance = stddev*stddev;
      double sampleVariance2 = 2 * variance*variance / (applyCount - 1);
      EXPECT_NEAR(boost::accumulators::variance(acc),
                  variance, 5*sqrt(sampleVariance2));
    }
  }

  // GAUSSIAN_QUANTIZED type
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian_quantized", 0, 0, 0, 0, 0));

    NoNoise(noise, applyCount);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
