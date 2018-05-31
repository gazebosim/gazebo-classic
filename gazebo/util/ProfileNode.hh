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
