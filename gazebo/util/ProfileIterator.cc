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


#include "gazebo/util/ProfileIterator.hh"
#include "gazebo/util/ProfileNode.hh"

namespace gazebo
{
  namespace util
  {
    ProfileIterator::ProfileIterator(ProfileNode * start)
    {
     CurrentParent = start;
     CurrentChild = CurrentParent->Get_Child();
    }

    void ProfileIterator::First(void)
    {
     CurrentChild = CurrentParent->Get_Child();
    }

    void ProfileIterator::Next(void)
    {
     CurrentChild = CurrentChild->Get_Sibling();
    }

    bool ProfileIterator::Is_Done(void)
    {
     return CurrentChild == NULL;
    }

    bool ProfileIterator::Is_Root(void)
    {
     return (CurrentParent->Get_Parent() == 0);
    }

    void ProfileIterator::Enter_Child(int index)
    {
     CurrentChild = CurrentParent->Get_Child();
     while ((CurrentChild != NULL) && (index != 0))
     {
      index--;
      CurrentChild = CurrentChild->Get_Sibling();
     }

     if (CurrentChild != NULL)
     {
      CurrentParent = CurrentChild;
      CurrentChild = CurrentParent->Get_Child();
     }
    }

    void ProfileIterator::Enter_Parent(void)
    {
     if (CurrentParent->Get_Parent() != NULL)
     {
      CurrentParent = CurrentParent->Get_Parent();
     }
     CurrentChild = CurrentParent->Get_Child();
    }

    const char * ProfileIterator::Get_Current_Name(void)
    {
      return CurrentChild->Get_Name();
    }

    int ProfileIterator::Get_Current_Total_Calls(void)
    {
      return CurrentChild->Get_Total_Calls();
    }

    float ProfileIterator::Get_Current_Total_Time(void)
    {
      return CurrentChild->Get_Total_Time();
    }

    // Access the current parent
    const char * ProfileIterator::Get_Current_Parent_Name(void)
    {
      return CurrentParent->Get_Name();
    }

    int ProfileIterator::Get_Current_Parent_Total_Calls(void)
    {
      return CurrentParent->Get_Total_Calls();
    }

    float ProfileIterator::Get_Current_Parent_Total_Time(void)
    {
      return CurrentParent->Get_Total_Time();
    }
  }
}
