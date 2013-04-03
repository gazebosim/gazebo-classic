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

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/MudPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MudPlugin)

/////////////////////////////////////////////////
MudPlugin::MudPlugin()
  : mudEnabled(false), hysteresis(0.2), newMsg(false)
{
}

/////////////////////////////////////////////////
void MudPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = this->model->GetWorld();
  this->link = _model->GetLink("link");

  GZ_ASSERT(_sdf, "MudPlugin _sdf pointer is NULL");
  if (_sdf->HasElement("contact_sensor_name"))
  {
    this->contactSensorName = _sdf->GetValueString("contact_sensor_name");

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());

    std::string topic = std::string("~/") + this->model->GetName() + "/" +
      this->contactSensorName;
    this->contactSub =
      this->node->Subscribe(topic, &MudPlugin::OnContact, this);
  }
  else
  {
    gzerr << "contactSensorName not supplied, ignoring contacts\n";
  }

  if (_sdf->HasElement("hysteresis"))
    this->hysteresis = _sdf->GetValueDouble("hysteresis");

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MudPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void MudPlugin::Init()
{
}

/////////////////////////////////////////////////
void MudPlugin::OnContact(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->newestContactsMsg = *_msg;
  this->newMsg = true;
}

/////////////////////////////////////////////////
void MudPlugin::OnUpdate()
{
  if (this->newMsg)
  {
    boost::mutex::scoped_lock lock(this->mutex);

    common::Time currTime = this->world->GetSimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;
    double physicsStep = this->world->GetPhysicsEngine()->GetMaxStepSize();
    double mySteps = stepTime.Double() / physicsStep;
    double contactStepRatio = 
      static_cast<double>(this->newestContactsMsg.contact_size()) / mySteps;

    // Use hysteresis based on contactStepRatio for enabling and disabling mud
    if (!this->joint && contactStepRatio > (0.5+hysteresis))
    {
      physics::LinkPtr link2;
      link2 = this->world->GetModel("unit_box_1")->GetLink("link");
      // Create the joint
      if (link2)
      {
        this->joint = this->world->GetPhysicsEngine()->CreateJoint(
          "revolute", this->model);

        this->joint->Attach(this->link, link2);
        this->joint->Load(this->link, link2, math::Pose());
        this->joint->SetName("mud_joint");

        this->joint->SetAttribute("erp", 0, 0.0);
        this->joint->SetAttribute("suspension_erp", 0, 0.0);
        this->joint->SetAttribute("stop_erp", 0, 0.0);

        this->joint->Init();

        this->joint->SetAttribute("erp", 0, 0.0);
        this->joint->SetAttribute("suspension_erp", 0, 0.0);
        this->joint->SetAttribute("stop_erp", 0, 0.0);

        gzerr << "erp " << this->joint->GetAttribute("erp", 0) << '\n';
        gzerr << "suspension_erp " << this->joint->GetAttribute("suspension_erp", 0) << '\n';
      }
    }
    else if (this->joint && contactStepRatio < (0.5-hysteresis))
    {
      bool paused = this->world->IsPaused();
      this->world->SetPaused(true);

      // reenable collision between the link pair
      physics::LinkPtr parent = this->joint->GetParent();
      physics::LinkPtr child = this->joint->GetChild();
      if (parent)
        parent->SetCollideMode("all");
      if (child)
        child->SetCollideMode("all");

      this->joint->Detach();
      this->joint.reset();

      this->world->SetPaused(paused);
    }
    
    // Debugging
    std::stringstream stream;
    stream << "mySteps " << mySteps << ' '
           << "contactStepRatio " << contactStepRatio << ' '
           << "this->joint " << this->joint << ' ';
    stream << ", now " << this->model->GetWorld()->GetSimTime() << '\n';;
    gzerr << stream.str();

    this->prevUpdateTime = currTime;
    this->newMsg = false;
  }
}
