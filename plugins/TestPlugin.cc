/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "physics/physics.hh"
#include "transport/transport.hh"
#include "plugins/TestPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(TestPlugin)

/////////////////////////////////////////////////
TestPlugin::TestPlugin()
{
}

/////////////////////////////////////////////////
void TestPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  physics::LinkPtr link1 = this->model->GetLink("link1");
  physics::LinkPtr link2 = this->model->GetLink("link2");
  physics::LinkPtr link3 = this->model->GetLink("link3");
  physics::LinkPtr link4 = this->model->GetLink("link4");
  physics::LinkPtr link5 = this->model->GetLink("link5");

  this->CreateJoint(link1, link2);
  this->CreateJoint(link2, link3);
  this->CreateJoint(link3, link4);
  this->CreateJoint(link4, link5);
}

void TestPlugin::CreateJoint(physics::LinkPtr _centerLink,
    physics::LinkPtr _extLink)
{
  math::Vector3 axis,direction;

  // compute the axis
  direction = _extLink->GetWorldPose().pos - _centerLink->GetWorldPose().pos;
  direction.Normalize();
  axis = direction.Cross(math::Vector3(0.0, 0.0, 1.0));

  this->myJoints.push_back(
      this->model->GetWorld()->GetPhysicsEngine()->CreateJoint(
        "revolute", this->model));

  this->myJoints.back()->Attach(_extLink, _centerLink );

  this->myJoints.back()->Load(_extLink, _centerLink,
      math::Pose(_centerLink->GetWorldPose().pos, math::Quaternion()));

  this->myJoints.back()->SetAxis(0, axis);

  /*this->myJoints.back()->SetAttribute("stop_cfm",0, this->stop_cfm);
  this->myJoints.back()->SetAttribute("stop_erp",0, this->stop_erp);
  this->myJoints.back()->SetAttribute("hi_stop",0, this->limit_stop);
  this->myJoints.back()->SetAttribute("lo_stop",0, - this->limit_stop);
  */
}
