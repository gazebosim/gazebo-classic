/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Random number generator
 * Author: Nate Koenig
 * Date: 27 May 2009
 * SVN: $Id:$
 */

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
