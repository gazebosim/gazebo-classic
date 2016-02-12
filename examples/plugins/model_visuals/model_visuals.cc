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
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class ModelVisuals : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      transport::PublisherPtr visPub;
      msgs::Visual visualMsg;

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(_parent->GetWorld()->GetName());
      visPub = this->node->Advertise<msgs::Visual>("~/visual", 10);

      // Set the visual's name. This should be unique.
      visualMsg.set_name("__RED_CYLINDER_VISUAL__");

      // Set the visual's parent. This visual will be attached to the parent
      visualMsg.set_parent_name(_parent->GetScopedName());

      // Create a cylinder
      msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
      geomMsg->set_type(msgs::Geometry::CYLINDER);
      geomMsg->mutable_cylinder()->set_radius(1);
      geomMsg->mutable_cylinder()->set_length(.1);

      // Set the material to be bright red
      visualMsg.mutable_material()->set_script("Gazebo/RedGlow");

      /// Set the pose of the visual relative to its parent
      msgs::Set(visualMsg.mutable_pose(), math::Pose(0, 0, 0.6, 0, 0, 0));

      // Don't cast shadows
      visualMsg.set_cast_shadows(false);

      visPub->Publish(visualMsg);
    }

    private: transport::NodePtr node;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelVisuals)
}
