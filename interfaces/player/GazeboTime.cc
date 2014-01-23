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
/* Desc: Gazebo Time functions
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 */

#include <math.h>
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"

#include "GazeboTime.hh"

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboTime::GazeboTime()
{
  this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
  this->statsSub =
    this->node->Subscribe("~/world_stats", &GazeboTime::OnStats, this);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboTime::~GazeboTime()
{
}

////////////////////////////////////////////////////////////////////////////////
void GazeboTime::OnStats(ConstWorldStatisticsPtr &_msg)
{
  this->simTime  = gazebo::msgs::Convert(_msg->sim_time());
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulator time
int GazeboTime::GetTime(struct timeval *_time)
{
  _time->tv_sec = this->simTime.sec;
  _time->tv_usec = this->simTime.nsec * 1e3;
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulator time
int GazeboTime::GetTimeDouble(double *_time)
{
  *_time = this->simTime.Double();
  return 0;
}
