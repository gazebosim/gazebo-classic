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
#include <boost/bind.hpp>
#include "physics/physics.h"
#include "gazebo.h"

namespace gazebo
{
  class RayTest : public Plugin
  {
    public: RayTest() : Plugin() {}

    public: void Load( sdf::ElementPtr &_sdf )
    {
      // Get then name of the parent model
      std::string modelName = _sdf->GetParent()->GetValueString("name");

      // Get the default world.
      physics::WorldPtr world = physics::get_world("default");

      // Get a pointer to the model
      this->model = world->GetModelByName(modelName);

      // Error message if the model couldn't be found
      if (!this->model)
        gzerr << "Unable to get parent model\n";

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStartSignal(
          boost::bind(&RayTest::OnUpdate, this));
      gzerr << "plugin model name: " << modelName << "\n";
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // do something on update
      gzerr << "plugin update\n";
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_PLUGIN(RayTest)
}
