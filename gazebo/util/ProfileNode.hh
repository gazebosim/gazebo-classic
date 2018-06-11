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
    /// \brief A node in the Profile Hierarchy Tree
    class ProfileNode {
      /// \brief Constructor.
      public: ProfileNode(const char * name, ProfileNode * parent);

      /// \brief Destructor.
      public: ~ProfileNode(void);

      /// \brief Get node matching name, or create
      public: ProfileNode * Get_Sub_Node(const char * name);

      /// \brief Get parent to this node
      public: ProfileNode * Get_Parent(void);

      /// \brief Get sibiling to this node
      public: ProfileNode * Get_Sibling(void);

      /// \brief Get child to this node
      public: ProfileNode * Get_Child(void);

      /// \brief Cleanup allocated memory
      public: void CleanupMemory(void);

      /// \brief Reset internal node state
      public: void Reset(void);

      /// \brief Start counters/timer for this node
      public: void Call(void);

      /// \brief Stop counters/timer for this node
      public: bool Return(void);

      /// \brief Get node name
      public: const char * Get_Name(void);

      /// \brief Get node calls
      public: int Get_Total_Calls(void);

      /// \brief Get node execution time
      public: float Get_Total_Time(void);

      protected: const char * Name;

      protected: int TotalCalls;

      protected: gazebo::common::Time TotalTime;

      protected: gazebo::common::Time StartTime;

      protected: int RecursionCounter;

      protected: ProfileNode * Parent;

      protected: ProfileNode * Child;

      protected: ProfileNode * Sibling;
    };
  }
}

#endif
