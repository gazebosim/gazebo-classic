/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

const unsigned int g_applyCount = 100;

// We will use 5 sigma (4e-5 chance of failure)
const double g_sigma = 5.0;

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
    EXPECT_EQ(noise.GetNoiseType(), sensors::Noise::NONE);
  }

  // GAUSSIAN type
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian", 0, 0, 0, 0, 0));
    EXPECT_EQ(noise.GetNoiseType(), sensors::Noise::GAUSSIAN);
  }

  // GAUSSIAN_QUANTIZED type
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian_quantized", 0, 0, 0, 0, 0));
    EXPECT_EQ(noise.GetNoiseType(), sensors::Noise::GAUSSIAN_QUANTIZED);
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
  // Use constant input and repeatedly add noise to it.
  double x = 42.0;

  // boost code from http://stackoverflow.com/questions/3534335
  boost::accumulators::accumulator_set<double,
    boost::accumulators::stats<boost::accumulators::tag::mean,
                               boost::accumulators::tag::variance > > acc;

  for (unsigned int i = 0; i < _count; ++i)
  {
    double y = _noise.Apply(x);
    acc(y);
  }

  // The sample mean should be near x+mean, with standard deviation of
  // stddev / sqrt(_count)
  // https://onlinecourses.science.psu.edu/stat414/node/167
  // We will use 5 sigma (4e-5 chance of failure)
  double mean = _noise.GetMean() + _noise.GetBias();
  double stddev = _noise.GetStdDev();
  double sampleStdDev = g_sigma*stddev / sqrt(_count);
  EXPECT_NEAR(boost::accumulators::mean(acc), x+mean, sampleStdDev);

  // The sample variance has the following variance:
  // 2 stddev^4 / (_count - 1)
  // en.wikipedia.org/wiki/Variance#Distribution_of_the_sample_variance
  // Again use 5 sigma
  double variance = stddev*stddev;
  double sampleVariance2 = 2 * variance*variance / (_count - 1);
  EXPECT_NEAR(boost::accumulators::variance(acc),
              variance, g_sigma*sqrt(sampleVariance2));
}

//////////////////////////////////////////////////
// Test noise application
TEST(NoiseTest, ApplyNone)
{
  sensors::Noise noise;
  noise.Load(NoiseSdf("none", 0, 0, 0, 0, 0));

  NoNoise(noise, g_applyCount);
}

TEST(NoiseTest, ApplyGaussian)
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

    NoNoise(noise, g_applyCount);
  }

  // GAUSSIAN with non-zero means and standard deviations, but no bias
  mean = 10.0;
  stddev = 5.0;
  biasMean = 0.0;
  biasStddev = 0.0;
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian", mean, stddev, biasMean, biasStddev, 0));
    EXPECT_NEAR(noise.GetBias(), 0.0, 1e-6);

    GaussianNoise(noise, g_applyCount);
  }

  // GAUSSIAN with non-zero mean, exact bias, and standard deviations
  mean = 10.0;
  stddev = 5.0;
  biasMean = 100.0;
  biasStddev = 0.0;
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian", mean, stddev, biasMean, biasStddev, 0));

    GaussianNoise(noise, g_applyCount);
  }

  // Test bias generation
  mean = 0.0;
  stddev = 0.0;
  biasMean = 0.0;
  biasStddev = 5.0;
  {
    boost::accumulators::accumulator_set<double,
      boost::accumulators::stats<boost::accumulators::tag::mean,
                                 boost::accumulators::tag::variance > > acc;

    for (unsigned int i = 0; i < g_applyCount; ++i)
    {
      sensors::Noise noise;
      noise.Load(NoiseSdf("gaussian", mean, stddev, biasMean, biasStddev, 0));

      acc(noise.GetBias());
    }

    // See comments in GaussianNoise function to explain these calculations.
    double sampleStdDev = g_sigma*biasStddev / sqrt(g_applyCount);
    EXPECT_NEAR(boost::accumulators::mean(acc), 0.0, sampleStdDev);

    double variance = biasStddev*biasStddev;
    double sampleVariance2 = 2 * variance*variance / (g_applyCount - 1);
    EXPECT_NEAR(boost::accumulators::variance(acc),
                variance, g_sigma*sqrt(sampleVariance2));
  }
}

TEST(NoiseTest, ApplyGaussianQuantized)
{
  double mean, stddev, biasMean, biasStddev, precision;

  // GAUSSIAN_QUANTIZED with zero means and standard deviations
  // should be the same as NONE
  mean = 0.0;
  stddev = 0.0;
  biasMean = 0.0;
  biasStddev = 0.0;
  precision = 0.0;
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian_quantized", mean, stddev, biasMean,
                        biasStddev, precision));

    NoNoise(noise, g_applyCount);
  }

  // GAUSSIAN_QUANTIZED with non-zero means and standard deviations,
  // but no bias or precision
  mean = 10.0;
  stddev = 5.0;
  biasMean = 0.0;
  biasStddev = 0.0;
  precision = 0.0;
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian_quantized", mean, stddev, biasMean,
                        biasStddev, precision));
    EXPECT_NEAR(noise.GetBias(), 0.0, 1e-6);

    GaussianNoise(noise, g_applyCount);
  }

  // GAUSSIAN with non-zero mean, exact bias, and standard deviations
  // no precision specified
  mean = 10.0;
  stddev = 5.0;
  biasMean = 100.0;
  biasStddev = 0.0;
  precision = 0.0;
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian_quantized", mean, stddev, biasMean,
                        biasStddev, precision));

    GaussianNoise(noise, g_applyCount);
  }

  // Test bias generation
  mean = 0.0;
  stddev = 0.0;
  biasMean = 0.0;
  biasStddev = 5.0;
  precision = 0.0;
  {
    boost::accumulators::accumulator_set<double,
      boost::accumulators::stats<boost::accumulators::tag::mean,
                                 boost::accumulators::tag::variance > > acc;

    for (unsigned int i = 0; i < g_applyCount; ++i)
    {
      sensors::Noise noise;
      noise.Load(NoiseSdf("gaussian_quantized", mean, stddev, biasMean,
                          biasStddev, precision));

      acc(noise.GetBias());
    }

    // See comments in GaussianNoise function to explain these calculations.
    double sampleStdDev = g_sigma*biasStddev / sqrt(g_applyCount);
    EXPECT_NEAR(boost::accumulators::mean(acc), 0.0, sampleStdDev);

    double variance = biasStddev*biasStddev;
    double sampleVariance2 = 2 * variance*variance / (g_applyCount - 1);
    EXPECT_NEAR(boost::accumulators::variance(acc),
                variance, g_sigma*sqrt(sampleVariance2));
  }

  // Test precision
  mean = 0.0;
  stddev = 0.0;
  biasMean = 0.0;
  biasStddev = 0.0;
  precision = 0.3;
  {
    sensors::Noise noise;
    noise.Load(NoiseSdf("gaussian_quantized", mean, stddev, biasMean,
                        biasStddev, precision));

    EXPECT_NEAR(noise.Apply(0.32), 0.3, 1e-6);
    EXPECT_NEAR(noise.Apply(0.31), 0.3, 1e-6);
    EXPECT_NEAR(noise.Apply(0.30), 0.3, 1e-6);
    EXPECT_NEAR(noise.Apply(0.29), 0.3, 1e-6);
    EXPECT_NEAR(noise.Apply(0.28), 0.3, 1e-6);

    EXPECT_NEAR(noise.Apply(-12.92), -12.9, 1e-6);
    EXPECT_NEAR(noise.Apply(-12.91), -12.9, 1e-6);
    EXPECT_NEAR(noise.Apply(-12.90), -12.9, 1e-6);
    EXPECT_NEAR(noise.Apply(-12.89), -12.9, 1e-6);
    EXPECT_NEAR(noise.Apply(-12.88), -12.9, 1e-6);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
