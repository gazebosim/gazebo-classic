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

/***************************************************************************************************
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

#ifndef _GAZEBO_UTIL_PROFILER_HH_
#define _GAZEBO_UTIL_PROFILER_HH_

#include "gazebo/gazebo_config.h"
#include "gazebo/util/ProfileManager.hh"

namespace gazebo
{
  namespace util
  {
    typedef void (EnterProfileZoneFunc)(const char* msg);
    typedef void (LeaveProfileZoneFunc)();

    EnterProfileZoneFunc* GetCurrentEnterProfileZoneFunc();
    LeaveProfileZoneFunc* GetCurrentLeaveProfileZoneFunc();

    void SetCustomEnterProfileZoneFunc(EnterProfileZoneFunc* enterFunc);
    void SetCustomLeaveProfileZoneFunc(LeaveProfileZoneFunc* leaveFunc);

    int GetCurrentThreadIndex2();
    constexpr int PROFILER_MAX_THREAD_COUNT = 64;
  }
}

class GZ_UTIL_VISIBLE ProfileSample
{
  public: explicit ProfileSample(const char * name);

  public: ~ProfileSample(void);
};


#ifdef ENABLE_PROFILER
#define GZ_PROFILE(name) ProfileSample __profile(name)
#else
#define GZ_PROFILE(name) ((void) 0)
#endif

#endif


