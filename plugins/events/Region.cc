/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <gazebo/common/Console.hh>
#include "plugins/events/Region.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
bool Region::Contains(const math::Vector3 &_p) const
{
  for (auto v: this->boxes)
  {
    if (v.Contains(_p))
    {
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
void Region::Load(const sdf::ElementPtr &_sdf)
{
  sdf::ElementPtr child = _sdf->GetFirstElement();
  while (child)
  {
    std::string ename = child->GetName();
    if (ename == "volume")
    {
      this->boxes.push_back(math::Box(child->Get<math::Vector3>("min"),
                                      child->Get<math::Vector3>("max")));
    }
    else if (ename == "name")
    {
      this->name = child->Get<std::string>();
    }
    else
    {
      gzwarn << "Unexpected element \"" + ename + "\" in Region element.";
    }
    child = child->GetNextElement();
  }
}
