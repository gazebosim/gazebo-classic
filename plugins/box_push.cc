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
#include "common/Plugin.hh"
#include "common/Console.hh"
#include "common/Events.hh"

#include "physics/Physics.hh"
#include "physics/PhysicsTypes.hh"
#include "physics/World.hh"
#include "physics/Base.hh"
#include "physics/Model.hh"

namespace gazebo
{
  namespace common
  {
    class BoxPush : public Plugin
    {
      public: BoxPush() : Plugin() 
      { }

      public: ~BoxPush()
      { }

      public: void Load( sdf::ElementPtr &_sdf )
      {
        std::string modelName = _sdf->GetParent()->GetValueString("name");

        physics::WorldPtr world = physics::get_world("default");
        this->model = world->GetModelByName(modelName);

        if (this->model)
        {
          gzdbg << "Model[" << this->model->GetName() << "]\n";
        }
        else
          gzerr << "Unable to get model\n";

        this->updateConnection = event::Events::ConnectWorldUpdateStartSignal(
            boost::bind(&BoxPush::UpdateCB, this));
      }

      public: void UpdateCB()
      {
        this->model->SetLinearVel( math::Vector3(.1, 0, 0) );
      }

      private: physics::ModelPtr model;
      private: event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_PLUGIN("BoxPush", BoxPush)
  }
}
