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

#include "common/CommonTypes.hh"
#include "common/Animation.hh"
#include "common/KeyFrame.hh"

#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"

namespace gazebo
{
  class PR2JointTest : public ModelPlugin
  {
    public: void Load(physics::ModelPtr &_parent, sdf::ElementPtr & /*_sdf*/)
    {
      // Get a pointer to the model
      this->model = _parent;

      // Error message if the model couldn't be found
      if (!this->model)
        gzerr << "Unable to get parent model\n";

      /*gazebo::common::NumericAnimationPtr shoulderAnim(
          new gazebo::common::NumericAnimation("shoulder_anim", 0.5, false));

      gazebo::common::NumericAnimationPtr headAnim(
          new gazebo::common::NumericAnimation("head_anim", 0.5, false));


      gazebo::common::NumericKeyFrame *key;
      key = shoulderAnim->CreateKeyFrame(0);
      key->SetValue(0);

      key = shoulderAnim->CreateKeyFrame(0.5);
      key->SetValue(-1.0);

      key = headAnim->CreateKeyFrame(0);
      key->SetValue(0);

      key = headAnim->CreateKeyFrame(0.5);
      key->SetValue(1.0);

      this->model->SetJointAnimation("head_pan_joint", headAnim);
      this->model->SetJointAnimation("r_shoulder_pan_joint", shoulderAnim);
      */
    }

    public: void Init()
    {
      std::map<std::string, double> positions;
      positions["r_shoulder_pan_joint"] = -0.330230;
      positions["r_shoulder_lift_joint"] = 0.008300;
      positions["r_upper_arm_roll_joint"] = -1.550000;
      positions["r_elbow_flex_joint"] = -0.859908;
      positions["r_forearm_roll_joint"] = 3.139403;
      positions["r_wrist_flex_joint"] = -0.529580;
      positions["r_wrist_roll_joint"] = -1.591270;

      this->model->SetJointPositions(positions);
    }

    private: physics::ModelPtr model;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PR2JointTest)
}

