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

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <gazebo/common/Assert.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include "plugins/ModelControlPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ModelControlPlugin)

/////////////////////////////////////////////////
ModelControlPlugin::ModelControlPlugin()
{
}

/////////////////////////////////////////////////
void ModelControlPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ModelControlPlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "ModelControlPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "ModelControlPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "ModelControlPlugin _sdf pointer is NULL");

  this->targetModel = this->world->GetModel("box");

  this->pelvisLink = this->model->GetLink("pelvis");

  this->endEffectorLink = this->model->GetLink("r_hand");

  this->joints = this->model->GetJoints();
  this->links = this->model->GetLinks();
  /*
  // get a list of links
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    while (elem)
    {
      // create a vector of empty links and joints
      links.push_back(this->model->GetLink(elem->Get<std::string>()));
      // joints.push_back(physics::JointPtr());
      elem = elem->GetNextElement("link_name");
    }
  }

  // get a list of joints
  if (_sdf->HasElement("joint_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("joint_name");
    while (elem)
    {
      // create a vector of empty joints and joints
      joints.push_back(this->model->GetJoint(elem->Get<std::string>()));
      // joints.push_back(physics::JointPtr());
      elem = elem->GetNextElement("joint_name");
    }
  }
  */
}

/////////////////////////////////////////////////
void ModelControlPlugin::Init()
{
  /*
  // subscribe to a topic
  std::string topicRequest = std::string("~/") + this->modelName + "/control_request";
  this->pubControlRequest =
    this->node->Advertise<msgs::ControlRequest>(topicRequest);

  std::string topicResponse = std::string("~/") + this->modelName + "/control_response";
  this->subControlResponse =
    this->node->Subscribe(topicResponse, &ModelControlPlugin::OnControlResponse, this);
  */

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelControlPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void ModelControlPlugin::OnControlResponse(gazebo::msgs::ControlResponse &_msg)
{
  // gzdbg << "Response: [" << _msg.DebugString() << "]" << std::endl;

  // pass response efforts to joints
  for (int i = 0; i < _msg.torques().size(); ++i)
  {
    /*
    gzerr << i << " : "
          << this->joints[this->jointId[i]]->GetName()
          << " : " << _msg.torques(i)
          << "\n";
    */
    this->joints[this->jointId[i]]->SetForce(0, _msg.torques(i));
  }
 
  /* 
  {
    // signal to continue simulation
    boost::mutex::scoped_lock lock(this->mutex);
    this->delayCondition.notify_one();
  }
  */
  
}

/////////////////////////////////////////////////
void ModelControlPlugin::RequestControl()
{
  // boost::mutex::scoped_lock lock(this->mutex); // for threading this...

  // TODO: [optional] get end-effector states (for noise rejection tests)

  // get joint states

  // publish request
  gazebo::msgs::ControlRequest req;
  req.set_name("request");

  req.clear_joint_names();
  req.clear_joint_pos();
  req.clear_joint_vel();
  this->jointId.clear();
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    // pluck pelvis and up, no legs
    if (this->joints[i]->GetName().find("_leg_") == std::string::npos)
    {
      this->jointId.push_back(i);
      req.add_joint_names(this->joints[i]->GetName());
      req.add_joint_pos(this->joints[i]->GetAngle(0).Radian());
      req.add_joint_vel(this->joints[i]->GetVelocity(0));
    }
  }

  // set pelvis pose
  gazebo::msgs::Set(req.mutable_pelvis_pose(),
    this->pelvisLink->GetWorldPose());

  // set end effector
  // offset from endEffectorLink from endEffectorLink origin
  //in endEffectorLink frame, this should match the
  // controller m_endEffectorStation in Atlas.cpp
  gazebo::math::Vector3 endEffectorStation(0, -.15, 0);
  // rotate local offset into world frame
  gazebo::math::Pose endEffectorPose = this->endEffectorLink->GetWorldPose();
  gazebo::math::Vector3 endEffectorPosWorld =
    endEffectorPose.rot.RotateVectorReverse(endEffectorStation) +
    endEffectorPose.pos;
  gazebo::msgs::Set(req.mutable_end_effector_pos(),
    endEffectorPosWorld);

  // set target position
  gazebo::msgs::Set(req.mutable_target_pos(),
    this->targetModel->GetWorldPose().pos);

  gazebo::msgs::ControlResponse res;
  bool result;
  // timeout in ms
  // if a response is not received within this time limit,
  // warn user via console messages and try again until
  // success.
  const int timeout = 1000;

  // copied from ign-transport example
  bool executed = false;
  while(!executed)
  {
    executed = this->node.Request(/*"/" + this->model->GetName() +*/
                                  "/control_request",
                                  req, timeout, res, result);
    if (executed)
    {
      if (result)
      {
        this->OnControlResponse(res);
      }
      else
        gzerr << "Service call failed" << std::endl;
    }
    else
      gzerr << "Service call timed out" << std::endl;
  }
}

/////////////////////////////////////////////////
void ModelControlPlugin::OnUpdate()
{
  // call every simulation step
  // this call blocks simulation
  this->RequestControl();
}
