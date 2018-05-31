#ifndef _GAZEBO_UTIL_PROFILENODE_HH_ 
#define _GAZEBO_UTIL_PROFILENODE_HH_ 

#include "gazebo/common/Time.hh"

namespace gazebo {
  namespace util {
    ///A node in the Profile Hierarchy Tree
    class ProfileNode {
    public:
      ProfileNode( const char * name, ProfileNode * parent );
      ~ProfileNode( void );

      ProfileNode * Get_Sub_Node( const char * name );
      ProfileNode * Get_Parent( void )  { return Parent; }
      ProfileNode * Get_Sibling( void )  { return Sibling; }
      ProfileNode * Get_Child( void )   { return Child; }

      void CleanupMemory();
      void Reset( void );
      void Call( void );
      bool Return( void );

      const char * Get_Name( void )    { return Name; }
      int Get_Total_Calls( void )  { return TotalCalls; }
      float Get_Total_Time( void )  { return TotalTime.Float() * 1000; }

    protected:
      const char * Name;
      int TotalCalls;
      gazebo::common::Time TotalTime;
      gazebo::common::Time StartTime;
      int RecursionCounter;

      ProfileNode * Parent;
      ProfileNode * Child;
      ProfileNode * Sibling;
    };
  }
}

#endif
