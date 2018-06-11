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


#include "gazebo/util/ProfileNode.hh"

namespace gazebo
{
  namespace util
  {
    ProfileNode::ProfileNode(const char * name, ProfileNode * parent) :
      Name(name),
      TotalCalls(0),
      TotalTime(0),
      StartTime(0),
      RecursionCounter(0),
      Parent(parent),
      Child(nullptr),
      Sibling(nullptr)
    {
      Reset();
    }

    ProfileNode::~ProfileNode(void)
    {
      CleanupMemory();
    }

    ProfileNode * ProfileNode::Get_Sub_Node(const char * name)
    {
      // Try to find this sub node
      ProfileNode * child = Child;
      while (child)
      {
        if (child->Name == name)
        {
          return child;
        }
        child = child->Sibling;
      }

      // We didn't find it, so add it

      ProfileNode * node = new ProfileNode(name, this);
      node->Sibling = Child;
      Child = node;
      return node;
    }

    ProfileNode * ProfileNode::Get_Parent(void)
    {
      return this->Parent;
    }

    ProfileNode * ProfileNode::Get_Sibling(void)
    {
      return this->Sibling;
    }

    ProfileNode * ProfileNode::Get_Child(void)
    {
      return this->Child;
    }

    void ProfileNode::CleanupMemory()
    {
      delete (Child);
      Child = nullptr;
      delete (Sibling);
      Sibling = nullptr;
    }

    void ProfileNode::Reset(void)
    {
      TotalCalls = 0;
      TotalTime = 0.0f;

      if (Child)
      {
        Child->Reset();
      }
      if (Sibling)
      {
        Sibling->Reset();
      }
    }

    void ProfileNode::Call(void)
    {
      TotalCalls++;
      if (RecursionCounter++ == 0)
      {
        StartTime.SetToWallTime();
      }
    }

    bool ProfileNode::Return(void)
    {
      if (--RecursionCounter == 0 && TotalCalls != 0)
      {
        auto cur_time = gazebo::common::Time::GetWallTime();
        auto delta_t = cur_time - StartTime;
        TotalTime += delta_t;
      }
      return (RecursionCounter == 0);
    }

    const char* ProfileNode::Get_Name(void)
    {
      return this->Name;
    }

    int ProfileNode::Get_Total_Calls(void)
    {
      return TotalCalls;
    }

    float ProfileNode::Get_Total_Time(void)
    {
      return TotalTime.Float() * 1000;
    }
  }
}
