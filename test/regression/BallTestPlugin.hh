/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef __GAZEBO_BALL_TEST_PLUGIN_HH__
#define __GAZEBO_BALL_TEST_PLUGIN_HH__

#include "common/common.h"
#include "physics/physics.h"
#include "transport/transport.h"
#include "gazebo.hh"

namespace gazebo
{
  class BallTestPlugin : public WorldPlugin
  {
    public: BallTestPlugin() {}

    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;
    }

    public: virtual void Init()
    {
      printf("Init\n");
      for (int i = 0; i < 100; ++i)
      {
        std::ostringstream newModelStr;
        newModelStr << "<gazebo version ='1.0'>"
          << "<model name ='ball_" << i << "'>"
          << "<origin pose='" << i*0.01 << " 0 " << 10 + i*2 << " 0 0 0'/>"
          << "<link name ='link'>"
          << "  <inertial mass ='0.5'/>"
          << "  <collision name ='collision'>"
          << "    <geometry>"
          << "      <sphere radius ='0.5'/>"
          << "    </geometry>"
          << "  </collision>"
          << "  <visual name ='visual' cast_shadows ='true'>"
          << "    <geometry>"
          << "      <sphere radius ='0.5'/>"
          << "    </geometry>"
          << "  </visual>"
          << "</link>"
          << "</model>"
          << "</gazebo>";

        sdf::SDF sdf;
        sdf.SetFromString(newModelStr.str());
        this->world->InsertModel(sdf);
      }
    }

    private: physics::WorldPtr world;
  };

  GZ_REGISTER_WORLD_PLUGIN(BallTestPlugin)
}
#endif
