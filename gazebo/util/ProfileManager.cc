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

#include <cmath>

#include "gazebo/util/ProfileManager.hh"
#include "gazebo/util/ProfileIterator.hh"
#include "gazebo/util/ProfileManagerPrivate.hh"

namespace gazebo
{
  namespace util
  {
    ProfileManager::ProfileManager():
      dataPtr(new ProfileManagerPrivate)
    {
      this->dataPtr->FrameCounter = 0;
      this->dataPtr->ResetTime = gazebo::common::Time::Zero;
    }

    ProfileManager::~ProfileManager()
    {
      for (int i = 0; i < PROFILER_MAX_THREAD_COUNT; i++)
      {
        this->dataPtr->gRoots[i].CleanupMemory();
      }
    }

    void ProfileManager::Start_Profile(const char * name)
    {
      int threadIndex = GetCurrentThreadIndex2();
      if ((threadIndex < 0) || threadIndex >= PROFILER_MAX_THREAD_COUNT)
        return;

      auto& current_node = this->dataPtr->gCurrentNodes[threadIndex];
      if (name != current_node->Get_Name())
      {
        current_node = current_node->Get_Sub_Node(name);
      }
      current_node->Call();
    }

    void ProfileManager::Stop_Profile(void)
    {
      int threadIndex = GetCurrentThreadIndex2();
      if ((threadIndex < 0) || threadIndex >= PROFILER_MAX_THREAD_COUNT)
        return;

      // Return will indicate whether we should back up to our parent (we may
      // be profiling a recursive function)
      auto& current_node = this->dataPtr->gCurrentNodes[threadIndex];
      if (current_node->Return())
      {
        current_node = current_node->Get_Parent();
      }
    }

    void ProfileManager::Reset(void)
    {
      int threadIndex = GetCurrentThreadIndex2();
      if ((threadIndex < 0) || threadIndex >= PROFILER_MAX_THREAD_COUNT)
        return;
      this->dataPtr->gRoots[threadIndex].Reset();
      this->dataPtr->gRoots[threadIndex].Call();
      this->dataPtr->FrameCounter = 0;
      this->dataPtr->ResetTime.SetToWallTime();
    }

    void ProfileManager::Increment_Frame_Counter(void)
    {
      this->dataPtr->FrameCounter++;
    }

    int ProfileManager::Get_Frame_Count_Since_Reset(void)
    {
      return this->dataPtr->FrameCounter;
    }

    float ProfileManager::Get_Time_Since_Reset(void)
    {
      auto cur_time = gazebo::common::Time::GetWallTime();
      auto delta_t = cur_time - this->dataPtr->ResetTime;
      return delta_t.Float() * 1000;
    }

    ProfileIterator * ProfileManager::Get_Iterator(int thread_id) {
      if ((thread_id < 0) || thread_id >= PROFILER_MAX_THREAD_COUNT)
        return nullptr;
      return new ProfileIterator(&this->dataPtr->gRoots[thread_id]);
    }

    ProfileIterator * ProfileManager::Get_Iterator(void)
    {
      return Get_Iterator(GetCurrentThreadIndex2());
    }

    void ProfileManager::Release_Iterator(ProfileIterator* iterator)
    {
      delete iterator;
    }

    void ProfileManager::dumpRecursive(ProfileIterator* profileIterator, int spacing)
    {
      profileIterator->First();

      if (profileIterator->Is_Done())
        return;

      float accumulated_time = 0;

      float parent_time = profileIterator->Is_Root() ?
        Get_Time_Since_Reset() :
        profileIterator->Get_Current_Parent_Total_Time();

      auto space = [](int spacing){
        for (int i = 0; i < spacing; i++) {
          fprintf(stderr, ".");
        }
      };

      int frames_since_reset = Get_Frame_Count_Since_Reset();

      space(spacing);
      fprintf(stderr, "----------------------------------\n");
      space(spacing);

      fprintf(stderr, "Profiling: %s (total running time: %.3f ms) ---\n", profileIterator->Get_Current_Parent_Name(), parent_time);

      float totalTime = 0.f;
      int numChildren = 0;

      for (int i = 0; !profileIterator->Is_Done(); i++,profileIterator->Next())
      {
        numChildren++;
        float current_total_time = profileIterator->Get_Current_Total_Time();
        accumulated_time += current_total_time;
        float fraction = fabsf(parent_time) > 1e-9 ? (current_total_time / parent_time) * 100 : 0.f;

        space(spacing);

        const char* name = profileIterator->Get_Current_Name();
        const float time_per_frame = (current_total_time / (double)frames_since_reset);
        const int total_calls = profileIterator->Get_Current_Total_Calls();

        fprintf(stderr, "%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)\n", i, name, fraction, time_per_frame, total_calls);
        totalTime += current_total_time;
        //recurse into children
      }

      space(spacing);

      fprintf(stderr, "%s (%.3f %%) :: %.3f ms\n", "Unaccounted:", fabsf(parent_time) > 1e-9 ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);

      for (int i=0;i<numChildren;i++)
      {
        profileIterator->Enter_Child(i);
        dumpRecursive(profileIterator,spacing+3);
        profileIterator->Enter_Parent();
      }
    }

    void ProfileManager::dumpAll()
    {
      ProfileIterator* profileIterator = nullptr;
      profileIterator = ProfileManager::Get_Iterator();
      dumpRecursive(profileIterator, 0);
      ProfileManager::Release_Iterator(profileIterator);
    }

    void ProfileManager::dumpAllThreads()
    {
      for (size_t ii =0 ; ii < PROFILER_MAX_THREAD_COUNT; ++ii) {
        ProfileIterator* profileIterator = nullptr;
        profileIterator = ProfileManager::Get_Iterator(ii);
        std::cout << "------Thread-" << ii << std::endl;
        dumpRecursive(profileIterator, 0);
        std::cout << std::endl;

        ProfileManager::Release_Iterator(profileIterator);
      }
    }

  }
}
