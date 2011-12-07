/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Gazebo Time functions
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 * CVS: $Id$
 */

#include <math.h>

#include "GazeboClient.hh"
#include "GazeboTime.hh"

using namespace libgazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboTime::GazeboTime()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboTime::~GazeboTime()
{
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulator time
int GazeboTime::GetTime(struct timeval* time)
{
  time->tv_sec = (int) floor(GazeboClient::sim->data->simTime);
  time->tv_usec = (int) floor(fmod((double)GazeboClient::sim->data->simTime, (double)1.0) * 1e6);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulator time
int GazeboTime::GetTimeDouble(double* time)
{
  struct timeval ts;

  ts.tv_sec = (int) floor(GazeboClient::sim->data->simTime);
  ts.tv_usec = (int) floor(fmod((double)GazeboClient::sim->data->simTime, (double)1.0) * 1e6);

  *time = ts.tv_sec + ts.tv_usec/1e6;

  return 0;
}
