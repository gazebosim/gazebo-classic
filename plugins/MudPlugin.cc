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
  GZ_ASSERT(_model, "MudPlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "MudPlugin world pointer is NULL");

  this->link = _model->GetLink("link");
  GZ_ASSERT(this->link, "MudPlugin link pointer is NULL");

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

    unsigned int nc = this->newestContactsMsg.contact_size();
    std::set<std::string> targetLinkNames;
    std::map<std::string, unsigned int> linkNameIndices;

    // If new contacts, then get the link names
    if (nc)
    {
      // Latest contact data is at the end of msg.contact() vector
      // So iterate from the end to the beginning
      common::Time latestContactTime =
        msgs::Convert(this->newestContactsMsg.contact(nc-1).time());

      // Try to find name of the other collision
      // If collision1() starts with this->modelName, then use collision2()
      std::string targetCollName =
        this->newestContactsMsg.contact(nc-1).collision1();
      if (0 == this->modelName.compare(0, this->modelName.length(),
               targetCollName))
      {
        targetCollName = this->newestContactsMsg.contact(nc-1).collision2();
      }
      std::string tmpLinkName = targetCollName.substr(0,
                                targetCollName.rfind("::"));
      targetLinkNames.insert(tmpLinkName);
      linkNameIndices[tmpLinkName] = nc-1;

      for (int i = nc-2; i >= 0 &&
           msgs::Convert(this->newestContactsMsg.contact(i).time())
            == latestContactTime; --i)
      {
        targetCollName =
          this->newestContactsMsg.contact(i).collision1();
        if (0 == this->modelName.compare(0, this->modelName.length(),
               targetCollName))
        {
          targetCollName = this->newestContactsMsg.contact(i).collision2();
        }
        tmpLinkName = targetCollName.substr(0,
                      targetCollName.rfind("::"));
        targetLinkNames.insert(tmpLinkName);
        linkNameIndices[tmpLinkName] = i;
      }
    }

    // If there's an existing joint
    if (this->joint)
    {
      // If the targetLink doesn't exist or
      // if it doesn't have its name in the most recent set,
      // then destroy the joint
      if (!this->targetLink || (targetLinkNames.end() ==
          targetLinkNames.find(this->targetLink->GetScopedName())))
      {
        gzerr << "Destroying mud joint\n";
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
      // Otherwise, update the anchor position
      else
      {
        math::Vector3 contactPositionAverage;
        // Find the index to the correct contact data structure
        unsigned int i = linkNameIndices[this->targetLink->GetScopedName()];
        if (i < nc)
        {
          unsigned int pc = this->newestContactsMsg.contact(i).position_size();
          // Add up all the contact point positions
          for (unsigned int j = 0; j < pc; ++j)
          {
            contactPositionAverage +=
              msgs::Convert(this->newestContactsMsg.contact(i).position(j));
          }
          // Then divide by numer of contact points
          contactPositionAverage *= 1.0 / static_cast<double>(pc);
          gzerr << this->targetLink->GetScopedName()
                << " anchor position "
                << contactPositionAverage
                << '\n';
          this->joint->SetAnchor(0, contactPositionAverage);
        }
        else
        {
          gzerr << "Error in linkNameIndices\n";
        }
      }
    }

    // If there's no joint
    // and there are targetLinkNames
    // Then create a joint with the first one
    if (!this->joint && !targetLinkNames.empty())
    {
      std::string targetLinkName = *(targetLinkNames.begin());
      std::string targetModelName = targetLinkName.substr(0,
                                    targetLinkName.rfind("::"));
      physics::ModelPtr targetModel = this->world->GetModel(targetModelName);
      if (targetModel)
        this->targetLink = targetModel->GetLink(targetLinkName);

      if (this->targetLink)
      {
        // Create the joint
        gzerr << "Creating a mud joint with " << targetLinkName << '\n';
        this->targetLink->SetAutoDisable(false);
        this->joint = this->world->GetPhysicsEngine()->CreateJoint(
          "revolute", this->model);

        this->joint->Attach(this->link, this->targetLink);
        // TODO set anchor position here, copy code for contactPositionAverage
        // from above.
        this->joint->Load(this->link, this->targetLink, math::Pose());
        this->joint->SetName("mud_joint");

        this->joint->SetAttribute("erp", 0, 0.0);
        this->joint->SetAttribute("cfm", 0, 0.1);
        this->joint->SetAttribute("stop_erp", 0, 0.0);
        this->joint->SetAttribute("stop_cfm", 0, 0.1);
        this->joint->SetHighStop(0, 0.0);
        this->joint->SetLowStop(0, 0.0);

        this->joint->Init();
      }
    }

    // // Each contacts message, may have more than one contact
    // // The first contact is usually the oldest, so take the last contact
    // math::Vector3 contactPositionAverage;
    // std::cout << "MudPlugin: " << currTime << ' ';
    // if (this->newestContactsMsg.contact_size())
    // {
    //   int i = this->newestContactsMsg.contact_size() - 1;
    //   std::cout << msgs::Convert(this->newestContactsMsg.contact(i).time()) << ' ';
    //   for (int j = 0; j < this->newestContactsMsg.contact(i).position_size(); ++j)
    //   {
    //     std::cout << msgs::Convert(this->newestContactsMsg.contact(i).position(j)) << ',';
    //     contactPositionAverage += msgs::Convert(this->newestContactsMsg.contact(i).position(j));
    //   }
    //   contactPositionAverage *= 1.0 / this->newestContactsMsg.contact(i).position_size();
    //   std::cout << '\n' << contactPositionAverage;
    // }
    // std::cout << '\n';

    // Use hysteresis based on contactStepRatio for enabling and disabling mud
    // if (!this->joint && contactStepRatio > (0.5+hysteresis))
    // {
    //   physics::LinkPtr link2;
    //   link2 = this->world->GetModel("unit_box_1")->GetLink("link");
    //   // Create the joint
    //   if (link2)
    //   {
    //     link2->SetAutoDisable(false);
    //     this->joint = this->world->GetPhysicsEngine()->CreateJoint(
    //       "revolute", this->model);

    //     this->joint->Attach(this->link, link2);
    //     this->joint->Load(this->link, link2, math::Pose());
    //     this->joint->SetName("mud_joint");

    //     this->joint->SetAttribute("erp", 0, 0.0);
    //     this->joint->SetAttribute("cfm", 0, 0.1);
    //     this->joint->SetAttribute("stop_erp", 0, 0.0);
    //     this->joint->SetAttribute("stop_cfm", 0, 0.1);
    //     this->joint->SetHighStop(0, 0.0);
    //     this->joint->SetLowStop(0, 0.0);

    //     this->joint->Init();
    //   }
    // }
    // else if (this->joint && contactStepRatio < (0.5-hysteresis))
    // {
    //   bool paused = this->world->IsPaused();
    //   this->world->SetPaused(true);

    //   // reenable collision between the link pair
    //   physics::LinkPtr parent = this->joint->GetParent();
    //   physics::LinkPtr child = this->joint->GetChild();
    //   if (parent)
    //     parent->SetCollideMode("all");
    //   if (child)
    //     child->SetCollideMode("all");

    //   this->joint->Detach();
    //   this->joint.reset();

    //   this->world->SetPaused(paused);
    // }
    
    // // Debugging
    // std::stringstream stream;
    // stream << "mySteps " << mySteps << ' '
    //        << "contactStepRatio " << contactStepRatio << ' '
    //        << "this->joint " << this->joint << ' ';
    // stream << ", now " << this->model->GetWorld()->GetSimTime() << '\n';;
    // //gzerr << stream.str();

    this->newMsg = false;
  }
}
