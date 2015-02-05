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
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class AnimatePose : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      gazebo::common::PoseAnimationPtr anim(
          new gazebo::common::PoseAnimation("test", 1000.0, true));

      gazebo::common::PoseKeyFrame *key;

      key = anim->CreateKeyFrame(0);
      key->SetTranslation(math::Vector3(0, 0, 0));
      key->SetRotation(math::Quaternion(0, 0, 0));

      key = anim->CreateKeyFrame(1000.0);
      key->SetTranslation(math::Vector3(5, 0, 0));
      key->SetRotation(math::Quaternion(0, 0, 1.5707));

      _parent->SetAnimation(anim);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatePose)
}
