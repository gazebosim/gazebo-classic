/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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


/* 
 * Taken from bulletphysics/bullet3
 *
 * Bullet Continuous Collision Detection and Physics Library
 * http://bulletphysics.org

 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:

 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. 
 *    If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

/*
***************************************************************************************************
**
** profile.cpp
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

#include "gazebo/gazebo_config.h"
#include "gazebo/util/Profiler.hh"

namespace gazebo {
  namespace util {

int GetCurrentThreadIndex2() {
  const int kNullIndex = ~0U;

#if defined(GAZEBO_THREAD_LOCAL)
  static THREADLOCAL int sThreadIndex = kNullIndex;
#elif defined(_WIN32)
  __declspec(thread) static int sThreadIndex = kNullIndex;
#else
  int sThreadIndex = 0;
  return -1;
#endif

  static int gThreadCounter = 0;

  if (sThreadIndex == kNullIndex) {
    sThreadIndex = gThreadCounter++;
  }
  return sThreadIndex;
}

#ifdef ENABLE_PROFILER
void EnterProfileZoneDefault(const char* name)
{
  ProfileManager::Instance()->Start_Profile( name ); 
}
void LeaveProfileZoneDefault()
{
	ProfileManager::Instance()->Stop_Profile(); 
}
#else
void	EnterProfileZoneDefault(const char* /*name*/){}
void	LeaveProfileZoneDefault(){}
#endif //BT_NO_PROFILE

static EnterProfileZoneFunc* gz_enterFunc = EnterProfileZoneDefault;
static LeaveProfileZoneFunc* gz_leaveFunc = LeaveProfileZoneDefault;

void EnterProfileZone(const char* name)
{
	(gz_enterFunc)(name);
}

void LeaveProfileZone()
{
	(gz_leaveFunc)();
}

EnterProfileZoneFunc* GetCurrentEnterProfileZoneFunc()
{
	return gz_enterFunc;
}

LeaveProfileZoneFunc* GetCurrentLeaveProfileZoneFunc()
{
	return gz_leaveFunc;
}

void SetCustomEnterProfileZoneFunc(EnterProfileZoneFunc* enterFunc)
{
	gz_enterFunc = enterFunc;
}
void SetCustomLeaveProfileZoneFunc(LeaveProfileZoneFunc* leaveFunc)
{
	gz_leaveFunc = leaveFunc;
}
}
}

ProfileSample::ProfileSample( const char * name )
{ 
  gazebo::util::EnterProfileZone(name);
}

ProfileSample::~ProfileSample( void )					
{ 
  gazebo::util::LeaveProfileZone();
}


