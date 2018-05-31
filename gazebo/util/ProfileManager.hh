#ifndef _GAZEBO_UTIL_PROFILEMANAGER_HH_
#define _GAZEBO_UTIL_PROFILEMANAGER_HH_

#include "gazebo/common/SingletonT.hh"

#include <memory>

namespace gazebo {
  namespace util {
    class ProfileIterator;
    class ProfileManagerPrivate;

    ///The Manager for the Profile system
    class GZ_UTIL_VISIBLE ProfileManager :
      public SingletonT<ProfileManager> 
    {
      private: ProfileManager();
      private: virtual ~ProfileManager();
      public: void Start_Profile( const char * name );
      public: void Stop_Profile( void );

      public: void CleanupMemory(void);
      public: void Reset( void );
      public: void Increment_Frame_Counter( void );
      public: int Get_Frame_Count_Since_Reset( void );
      public: float Get_Time_Since_Reset( void );

      public: ProfileIterator * Get_Iterator( void ); 
      public: void Release_Iterator( ProfileIterator * iterator );

      public: void dumpRecursive(ProfileIterator* profileIterator, int spacing);
      public: void dumpAll();

      private: friend class SingletonT<ProfileManager>;
      private: std::unique_ptr<ProfileManagerPrivate> dataPtr;
    };
  }
}
#endif
