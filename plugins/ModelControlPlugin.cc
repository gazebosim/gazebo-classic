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

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/ContactSensor.hh"
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
  gzdbg << "Response: [" << _msg.DebugString() << "]" << std::endl;

  // pass response efforts to joints
  for (int i = 0; i < _msg.torques().size(); ++i)
  {
    this->joints[i]->SetForce(0, _msg.torques(i));
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
void ModelControlPlugin::PubControlRequest()
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
  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    req.add_joint_names(this->joints[i]->GetName());
    req.add_joint_pos(this->joints[i]->GetAngle(0).Radian());
    req.add_joint_vel(this->joints[i]->GetVelocity(0));
  }
  gazebo::msgs::Set(req.mutable_target_pos(),
    this->targetModel->GetWorldPose().pos);

  gazebo::msgs::ControlResponse res;
  bool result;
  const int timeout = 1000;  // in ms

  bool executed = false;
  while(!executed)
  {
    executed = this->node.Request("/" + this->model->GetName() + "/control_request",
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
  this->PubControlRequest();

  /*
  {
    boost::mutex::scoped_lock lock(this->mutex);
    // block simulation,
    // wait for a response from responser (task space controller)
    // calculate amount of time to wait based on rules
    boost::system_time timeout = boost::get_system_time();
    timeout += boost::posix_time::microseconds(1000);

    while (!this->delayCondition.timed_wait(lock, timeout))
    {
      this->PubControlRequest();
      // change delay to 1 sec
      timeout = boost::get_system_time() + boost::posix_time::microseconds(1000000);
      gzerr << "controller synchronization timedout: "
               "delay budget exhausted.\n";
    }
  }
  */
}
