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
/* Desc: Gazebo (simulator) client functions
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 */

#include <assert.h>
#include <stdio.h>
#include <iostream>
#include <libplayercore/playercore.h>

#include "GazeboTime.hh"
#include "GazeboClient.hh"

extern PlayerTime* GlobalTime;

void GazeboClient::Init(int /*_serverid*/, const std::string & /*_worldName*/)
{
  // steal the global clock - a bit aggressive, but a simple approach
  if (GlobalTime)
  {
    delete GlobalTime;
    GlobalTime = NULL;
  }

  GlobalTime = new GazeboTime();
  assert(GlobalTime != 0);
}

void GazeboClient::Fini()
{
}
