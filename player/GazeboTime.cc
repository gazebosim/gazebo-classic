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
/* Desc: Gazebo Time functions 
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 * CVS: $Id$
 */

#include <math.h>

#include "GazeboClient.hh"
#include "GazeboTime.hh"

using namespace gazebo;

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
  time->tv_sec = (int) floor(GazeboClient::sim->data->sim_time);
  time->tv_usec = (int) floor(fmod(GazeboClient::sim->data->sim_time, 1.0) * 1e6);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulator time
int GazeboTime::GetTimeDouble(double* time)
{
  struct timeval ts;

  ts.tv_sec = (int) floor(GazeboClient::sim->data->sim_time);
  ts.tv_usec = (int) floor(fmod(GazeboClient::sim->data->sim_time, 1.0) * 1e6);

  *time = ts.tv_sec + ts.tv_usec/1e6;

  return 0;
}
