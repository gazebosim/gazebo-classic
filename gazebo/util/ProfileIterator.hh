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

#ifndef _GAZEBO_UTIL_PROFILEITERATOR_HH_
#define _GAZEBO_UTIL_PROFILEITERATOR_HH_

namespace gazebo
{
  namespace util
  {
    class ProfileManager;
    class ProfileNode;

    /// \brief An iterator to navigate through the tree
    class ProfileIterator
    {
      // Access all the children of the current parent
      public: void First(void);
      public: void Next(void);
      public: bool Is_Done(void);
      public: bool Is_Root(void);

      // Make the given child the new parent
      public: void Enter_Child(int index);

      // Make the largest child the new parent
      public: void Enter_Largest_Child(void);

      // Make the current parent's parent the new parent
      public: void Enter_Parent(void);

      // Access the current child
      public: const char * Get_Current_Name(void);
      public: int Get_Current_Total_Calls(void);
      public: float Get_Current_Total_Time(void);

      // Access the current parent
      public: const char * Get_Current_Parent_Name(void);
      public: int Get_Current_Parent_Total_Calls(void);
      public: float Get_Current_Parent_Total_Time(void);

      protected: ProfileNode * CurrentParent;
      protected: ProfileNode * CurrentChild;
      protected: explicit ProfileIterator(ProfileNode * start);

      protected: friend class ProfileManager;
    };
  }
}

#endif
