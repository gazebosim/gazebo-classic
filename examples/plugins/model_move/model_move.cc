/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class ModelMove : public ModelPlugin
  {
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // PARAMETERS
      math::Vector3 start_point = math::Vector3(0, 0, 0);
      math::Vector3 goal_point = math::Vector3(0, 10, 0);
      
      math::Vector3 diff = goal_point - start_point;
      int anim_time = floor(diff.GetMax());
      float x_step = diff.x /anim_time;
      float y_step = diff.y /anim_time;
      float z_step = diff.z /anim_time;

      // create the animation
      gazebo::common::PoseAnimationPtr anim(
					    // name the animation "test",
					    // make it last 10 seconds,
					    // and set it on a repeat loop
					    new gazebo::common::PoseAnimation("test", anim_time, false));
      
      gazebo::common::PoseKeyFrame *key;
      
      // set starting location of the box
      key = anim->CreateKeyFrame(0);
      key->SetTranslation(math::Vector3(0, 0, 0));
      key->SetRotation(math::Quaternion(0, 0, 0));

      for (int i=1; i <= anim_time; i++) {
	key = anim->CreateKeyFrame(i);
	key->SetTranslation(math::Vector3(x_step*i, y_step*i, z_step*i));
	key->SetRotation(math::Quaternion(0, 0, 0));
      }
      
      // set the animation
      _parent->SetAnimation(anim);
    }
    
    // Pointer to the model
  private: physics::ModelPtr model;

    // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelMove);
}
