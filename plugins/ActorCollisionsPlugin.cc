/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "plugins/ActorCollisionsPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorCollisionsPlugin)


/////////////////////////////////////////////////
ActorCollisionsPlugin::ActorCollisionsPlugin()
{
}

/////////////////////////////////////////////////
void ActorCollisionsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  physics::ActorPtr actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

  ignition::math::Vector3d scaling = ignition::math::Vector3d::One;

  // Read in the collision scaling factor
  if (_sdf->HasElement("scaling"))
    scaling = _sdf->Get<ignition::math::Vector3d>("scaling");

  // Scale all the collisions
  for (const auto &link : actor->GetLinks())
  {
    // Init the links, which in turn enables collisions
    link->Init();
    for (const auto &collision : link->GetCollisions())
    {
      gazebo::physics::BoxShapePtr boxShape =
        boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(
            collision->GetShape());
      boxShape->SetSize(boxShape->Size() * scaling);
    }
  }
}

