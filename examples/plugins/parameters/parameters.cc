/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class Parameters : public WorldPlugin
  {
    public: void Load(physics::WorldPtr /*_parent*/, sdf::ElementPtr _sdf)
    {
      this->PrintValues("", _sdf);
    }

    private: void PrintValues(std::string _prefix, sdf::ElementPtr _sdf)
    {
      if (_sdf->GetValue())
        std::cout << _prefix << _sdf->GetValue()->GetAsString() << "\n";

      _prefix += "  ";
      sdf::ElementPtr elem = _sdf->GetFirstElement();
      while (elem)
      {
        this->PrintValues(_prefix, elem);
        elem = elem->GetNextElement();
      }
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Parameters)
}
