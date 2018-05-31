#include "gazebo/util/ProfileManager.hh"
#include "gazebo/util/ProfileIterator.hh"
#include "gazebo/util/ProfileManagerPrivate.hh"

#include <cmath>

namespace gazebo {
  namespace util {
    ProfileManager::ProfileManager():
      dataPtr(new ProfileManagerPrivate)
    {
      this->dataPtr->FrameCounter = 0;
      this->dataPtr->ResetTime = gazebo::common::Time::Zero;
    }

    ProfileManager::~ProfileManager() {
      this->CleanupMemory();
    }

    void ProfileManager::Start_Profile( const char * name )
    {
      int threadIndex = GetCurrentThreadIndex2();
      if ((threadIndex<0) || threadIndex >= PROFILER_MAX_THREAD_COUNT)
        return;

      if (name != this->dataPtr->gCurrentNodes[threadIndex]->Get_Name()) {
        this->dataPtr->gCurrentNodes[threadIndex] = this->dataPtr->gCurrentNodes[threadIndex]->Get_Sub_Node( name );
      }
      this->dataPtr->gCurrentNodes[threadIndex]->Call();
    }

    void ProfileManager::Stop_Profile( void )
    {
      int threadIndex = GetCurrentThreadIndex2();
      if ((threadIndex<0) || threadIndex >= PROFILER_MAX_THREAD_COUNT)
        return;

      // Return will indicate whether we should back up to our parent (we may
      // be profiling a recursive function)
      if (this->dataPtr->gCurrentNodes[threadIndex]->Return()) {
        this->dataPtr->gCurrentNodes[threadIndex] = this->dataPtr->gCurrentNodes[threadIndex]->Get_Parent();
      }
    }

    void ProfileManager::CleanupMemory(void)
    {
      for (int i=0;i<PROFILER_MAX_THREAD_COUNT;i++)
      {
        this->dataPtr->gRoots[i].CleanupMemory();
      }
    }

    void ProfileManager::Reset( void )
    {
      int threadIndex = GetCurrentThreadIndex2();
      if ((threadIndex<0) || threadIndex >= PROFILER_MAX_THREAD_COUNT)
        return;
      this->dataPtr->gRoots[threadIndex].Reset();
      this->dataPtr->gRoots[threadIndex].Call();
      this->dataPtr->FrameCounter = 0;
      this->dataPtr->ResetTime.SetToWallTime();
    }

    void ProfileManager::Increment_Frame_Counter( void )
    {
      this->dataPtr->FrameCounter++;
    }

    int ProfileManager::Get_Frame_Count_Since_Reset( void )  
    { 
      return this->dataPtr->FrameCounter; 
    }

    float ProfileManager::Get_Time_Since_Reset( void )
    {
      auto cur_time = gazebo::common::Time::GetWallTime();
      auto delta_t = cur_time - this->dataPtr->ResetTime;
      return delta_t.Float() * 1000; 
    }

    ProfileIterator *	ProfileManager::Get_Iterator( void )
    { 
        int threadIndex = GetCurrentThreadIndex2();
        if ((threadIndex<0) || threadIndex >= PROFILER_MAX_THREAD_COUNT)
          return 0;
        return new ProfileIterator( &this->dataPtr->gRoots[threadIndex]); 
    }

    void ProfileManager::Release_Iterator(ProfileIterator* iterator) {
      delete iterator;
    }

    void ProfileManager::dumpRecursive(ProfileIterator* profileIterator, int spacing)
    {
      profileIterator->First();

      if (profileIterator->Is_Done())
        return;

      float accumulated_time=0;

      float parent_time;
      if (profileIterator->Is_Root()) {
        parent_time = Get_Time_Since_Reset();
      } else {
        parent_time = profileIterator->Get_Current_Parent_Total_Time();
      }

      int frames_since_reset = Get_Frame_Count_Since_Reset();
      for (int i=0;i<spacing;i++)	{
        printf(".");
      }

      printf("----------------------------------\n");

      for (int i=0;i<spacing;i++)	printf(".");
      printf("Profiling: %s (total running time: %.3f ms) ---\n",	profileIterator->Get_Current_Parent_Name(), parent_time );
      float totalTime = 0.f;


      int numChildren = 0;

      for (int i = 0; !profileIterator->Is_Done(); i++,profileIterator->Next())
      {
        numChildren++;
        float current_total_time = profileIterator->Get_Current_Total_Time();
        accumulated_time += current_total_time;
        float fraction = fabsf(parent_time) > 1e-9 ? (current_total_time / parent_time) * 100 : 0.f;

        for (int j=0;j<spacing;j++)	{
          printf(".");
        }
        printf("%d -- %s (%.2f %%) :: %.3f ms / frame (%d calls)\n",i, profileIterator->Get_Current_Name(), fraction,(current_total_time / (double)frames_since_reset),profileIterator->Get_Current_Total_Calls());
        totalTime += current_total_time;
        //recurse into children
      }

      for (int i=0;i<spacing;i++)	printf(".");
      printf("%s (%.3f %%) :: %.3f ms\n", "Unaccounted:", fabsf(parent_time) > 1e-9 ? ((parent_time - accumulated_time) / parent_time) * 100 : 0.f, parent_time - accumulated_time);

      for (int i=0;i<numChildren;i++)
      {
        profileIterator->Enter_Child(i);
        dumpRecursive(profileIterator,spacing+3);
        profileIterator->Enter_Parent();
      }
    }

    void	ProfileManager::dumpAll()
    {
      ProfileIterator* profileIterator = 0;
      profileIterator = ProfileManager::Get_Iterator();
      dumpRecursive(profileIterator,0);
      ProfileManager::Release_Iterator(profileIterator);
    }

  }
}
