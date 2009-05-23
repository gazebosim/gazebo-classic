#include <ctime>
#include "Rand.hh"

using namespace gazebo;

GeneratorType *Rand::randGenerator = new GeneratorType(std::time(0));

///////////////////////////////////////////////////////////////////////////////
// Constructor
Rand::Rand()
{
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
Rand::~Rand()
{
}

///////////////////////////////////////////////////////////////////////////////
/// Get a double from a uniform distribution
double Rand::GetDblUniform(double min, double max)
{
  URealGen gen(*randGenerator, UniformRealDist(min, max));
 
  return gen(); 
}

///////////////////////////////////////////////////////////////////////////////
/// Get a double from a normal distribution
double Rand::GetDblNormal(double mean, double sigma)
{
  NRealGen gen(*randGenerator, NormalRealDist(mean, sigma));
 
  return gen(); 
}

///////////////////////////////////////////////////////////////////////////////
/// Get a integer from a uniform distribution
int Rand::GetIntUniform(int min, int max)
{
  UIntGen gen(*randGenerator, UniformIntDist(min,max));
 
  return gen(); 
}

///////////////////////////////////////////////////////////////////////////////
/// Get a double from a normal distribution
int Rand::GetIntNormal(int mean, int sigma)
{
  NRealGen gen(*randGenerator, NormalRealDist(mean, sigma));
 
  return (int)(gen()); 
}
