/* Copyright 2011 Nate Koenig & Andrew Howard
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

#include "common/common.h"
#include "transport/transport.h"
#include "physics/physics.h"
#include "math/Pose.hh"

#include "pioneer2dx.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MoviePioneer2dx)

/////////////////////////////////////////////////
MoviePioneer2dx::MoviePioneer2dx()
{
}

/////////////////////////////////////////////////
void MoviePioneer2dx::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->node = transport::NodePtr(new transport::Node());
}

/////////////////////////////////////////////////
void MoviePioneer2dx::Init()
{
  this->node->Init();
  this->poseAnimSub = this->node->Subscribe("~/pose_animation",
      &MoviePioneer2dx::OnPoseAnimation, this);

  this->world = physics::get_world("default");
  this->pioneer2dx = this->world->GetModel("pioneer2dx");

  if (!this->pioneer2dx)
    gzerr << "Unable to find pioneer2dx\n";

  // this->connections.push_back( event::Events::ConnectWorldUpdateStart(
  //      boost::bind(&MoviePioneer2dx::OnUpdate, this) ) );

}

/////////////////////////////////////////////////
/*void MoviePioneer2dx::OnUpdate()
{
}*/

/////////////////////////////////////////////////
void MoviePioneer2dx::OnPoseAnimation(ConstPoseAnimationPtr &_anim)
{
  if (_anim->model_name() == "pioneer2dx")
  {
    common::PoseAnimationPtr anim(new common::PoseAnimation("move", 20, false));

    double stepSize = 20.0 / _anim->pose_size();
    for (unsigned int i=0; i < _anim->pose_size(); ++i)
    {
      double time = i*stepSize;
      common::PoseKeyFrame *key = anim->CreateKeyFrame(
          msgs::Convert(_anim->time(i)).Double());
      key->SetTranslation(msgs::Convert(_anim->pose(i).position()));
      key->SetRotation(msgs::Convert(_anim->pose(i).orientation()));
    }

    this->pioneer2dx->SetAnimation(anim);
  }
}
