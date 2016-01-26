/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  class CameraMove : public ModelPlugin
  {
    public: CameraMove() : ModelPlugin() {}

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Get a pointer to the model
      this->model = _model;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&CameraMove::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      math::Vector3 v(0.03, 0, 0);
      math::Pose pose = this->model->GetWorldPose();
      v = pose.rot * v;

      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(v);
      this->model->SetAngularVel(math::Vector3(0, 0, 0.01));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CameraMove)
}
