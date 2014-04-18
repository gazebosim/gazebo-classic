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
#include <map>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
  class AnimateJoints : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      std::map<std::string, common::NumericAnimationPtr> anim;

      // Create a new animation for the "my_joint" define in the SDF file.
      // The animation will last for 5.0 seconds, and it will repeat
      anim["my_joint"].reset(new common::NumericAnimation(
            "my_animation", 5.0, true));

      // Create a key frame for the starting position of the joint
      common::NumericKeyFrame *key = anim["my_joint"]->CreateKeyFrame(0.0);
      key->SetValue(0.1);

      // Create a key frame half-way through the animation
      key = anim["my_joint"]->CreateKeyFrame(2.5);
      key->SetValue(2.0);

      // Create the end key frame to be at the same position as the start
      // for a smooth animation
      key = anim["my_joint"]->CreateKeyFrame(5.0);
      key->SetValue(0.1);

     // Attach the animation to the model
      _model->SetJointAnimation(anim);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimateJoints)
}
