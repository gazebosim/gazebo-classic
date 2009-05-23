#ifndef RAND_HH
#define RAND_HH

#include <boost/random.hpp>

namespace gazebo
{

  typedef boost::mt19937 GeneratorType;
  typedef boost::uniform_real<double> UniformRealDist;
  typedef boost::normal_distribution<double> NormalRealDist;
  typedef boost::uniform_int<int> UniformIntDist;

  typedef boost::variate_generator<GeneratorType&, UniformRealDist > URealGen;
  typedef boost::variate_generator<GeneratorType&, NormalRealDist > NRealGen;
  typedef boost::variate_generator<GeneratorType&, UniformIntDist > UIntGen;

  /// \brief Random number generator class
  class Rand
  {
    /// \brief Constructor
    private: Rand();

    /// \brief Destructor
    private: virtual ~Rand();
 
    /// \brief Get a double from a uniform distribution
    /// \param min Minimum bound for the random number
    /// \param max Maximum bound for the random number
    public: static double GetDblUniform(double min=0, double max=1);

    /// \brief Get a double from a normal distribution
    /// \param mean Mean value for the distribution
    /// \param sigma Sigma value for the distribution
    public: static double GetDblNormal(double mean=0, double sigma=1);

    /// \brief Get a integer from a uniform distribution
    /// \param min Minimum bound for the random number
    /// \param max Maximum bound for the random number
    public: static int GetIntUniform(int min, int max);

    /// \brief Get a double from a normal distribution
    /// \param mean Mean value for the distribution
    /// \param sigma Sigma value for the distribution
    public: static int GetIntNormal(int mean, int sigma);
  
    // The random number generator
    private: static GeneratorType *randGenerator;
  };
  
}

#endif
