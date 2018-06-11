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

#ifndef _GAZEBO_UTIL_PROFILEMANAGER_HH_
#define _GAZEBO_UTIL_PROFILEMANAGER_HH_

#include <memory>

#include "gazebo/common/SingletonT.hh"

namespace gazebo
{
  namespace util
  {
    class ProfileIterator;
    class ProfileManagerPrivate;

    /// \brief The Manager for the Profile system
    class GZ_UTIL_VISIBLE ProfileManager :
      public SingletonT<ProfileManager>
    {
      private: ProfileManager();

      private: virtual ~ProfileManager();

      public: void Start_Profile(const char * name);

      public: void Stop_Profile(void);

      public: void Reset(void);

      public: void Increment_Frame_Counter(void);

      public: int Get_Frame_Count_Since_Reset(void);

      public: float Get_Time_Since_Reset(void);

      public: ProfileIterator * Get_Iterator(int thread_id);

      public: ProfileIterator * Get_Iterator(void);

      public: void Release_Iterator(ProfileIterator * iterator);

      public: void dumpRecursive(ProfileIterator* profileIterator, int spacing);

      public: void dumpAll();

      public: void dumpAllThreads();

      private: friend class SingletonT<ProfileManager>;

      private: std::unique_ptr<ProfileManagerPrivate> dataPtr;
    };
  }
}
#endif
