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

#include <boost/algorithm/string.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/ContactSensor.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/MudPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MudPlugin)

/////////////////////////////////////////////////
MudPlugin::MudPlugin()
  : newMsg(false), newMsgWait(0), stiffness(0.0), damping(100.0),
    contactSurfaceBitmask(0)
{
}

/////////////////////////////////////////////////
void MudPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "MudPlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "MudPlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "MudPlugin physics pointer is NULL");

  this->link = _model->GetLink();
  GZ_ASSERT(this->link, "MudPlugin link pointer is NULL");

  GZ_ASSERT(_sdf, "MudPlugin _sdf pointer is NULL");
  if (_sdf->HasElement("contact_sensor_name"))
  {
    this->contactSensorName = _sdf->Get<std::string>("contact_sensor_name");
  }
  else
  {
    gzerr << "contactSensorName not supplied, ignoring contacts\n";
  }

  if (_sdf->HasElement("stiffness"))
    this->stiffness = _sdf->Get<double>("stiffness");

  if (_sdf->HasElement("damping"))
    this->damping = _sdf->Get<double>("damping");

  if (_sdf->HasElement("contact_surface_bitmask"))
  {
    this->contactSurfaceBitmask =
        _sdf->Get<unsigned int>("contact_surface_bitmask");
  }

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    while (elem)
    {
      allowedLinks.push_back(elem->Get<std::string>());
      links.push_back(physics::LinkPtr());
      joints.push_back(physics::JointPtr());
      elem = elem->GetNextElement("link_name");
    }
  }
  GZ_ASSERT(allowedLinks.size() == links.size(),
    "Length of links data structure doesn't match allowedLinks");
  GZ_ASSERT(allowedLinks.size() == joints.size(),
    "Length of joints data structure doesn't match allowedLinks");
}

/////////////////////////////////////////////////
void MudPlugin::Init()
{
  this->node.reset(new transport::Node());
  this->node->Init(this->world->GetName());

  if (!this->contactSensorName.empty())
  {
    std::string topic = std::string("~/") + this->modelName + "/" +
      this->contactSensorName;
    this->contactSub =
      this->node->Subscribe(topic, &MudPlugin::OnContact, this);

    // create bitmask from contact sensor's collisions if it's not specified in
    // the sdf
    if (!this->sdf->HasElement("contact_surface_bitmask"))
    {
      std::string name = this->contactSensorName;
      boost::replace_all(name, "/", "::");
      name = this->world->GetName() + "::"+ this->modelName + "::" + name;
      sensors::SensorManager *mgr = sensors::SensorManager::Instance();
      // Get a pointer to the contact sensor
      sensors::ContactSensorPtr sensor =
          boost::dynamic_pointer_cast<sensors::ContactSensor>
          (mgr->GetSensor(name));
      if (sensor)
      {
        for (unsigned int i = 0; i < sensor->GetCollisionCount(); ++i)
        {
          std::string colName = sensor->GetCollisionName(i);
          physics::CollisionPtr colPtr =
              boost::dynamic_pointer_cast<physics::Collision>(
              this->world->GetEntity(colName));
          if (colPtr)
          {
            this->contactSurfaceBitmask |=
                colPtr->GetSurface()->collideWithoutContactBitmask;
          }
        }
      }
      else
      {
        gzerr << "Unable to GetSensor, ignoring contact_surface_bitmask\n";
      }
    }
  }

  for (unsigned int i = 0; i < this->allowedLinks.size(); ++i)
  {
    physics::LinkPtr allowedLink = boost::dynamic_pointer_cast<physics::Link>(
        this->world->GetEntity(this->allowedLinks[i]));

    if (!allowedLink)
      continue;

    std::vector<physics::CollisionPtr> collisions
        = allowedLink->GetCollisions();
    for (unsigned int j = 0; j < collisions.size(); ++j)
    {
      collisions[j]->GetSurface()->collideWithoutContactBitmask |=
          this->contactSurfaceBitmask;
    }
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MudPlugin::OnUpdate, this));
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
  double dt = this->physics->GetMaxStepSize();
  if (dt < 1e-6)
    dt = 1e-6;
  if (this->newMsg)
  {
    boost::mutex::scoped_lock lock(this->mutex);

    unsigned int nc = this->newestContactsMsg.contact_size();
    boost::unordered_set<std::string> contactLinkNames;
    boost::unordered_map<std::string, unsigned int> linkNameIndices;

    // If new contacts, then get the link names
    if (nc)
    {
      // Starting with last contact message, iterate backwards
      // checking each contact with a timestamp matching the last contact
      // Add each link name to contactLinkNames
      common::Time latestContactTime =
        msgs::Convert(this->newestContactsMsg.contact(nc-1).time());
      std::string targetCollName, tmpLinkName;

      for (int i = nc-1; i >= 0 &&
           msgs::Convert(this->newestContactsMsg.contact(i).time())
            == latestContactTime; --i)
      {
        // Try to find name of the other collision
        // If collision1() starts with this->modelName, then use collision2()
        targetCollName = this->newestContactsMsg.contact(i).collision1();
        if (0 == targetCollName.compare(0, this->modelName.length(),
               targetCollName))
        {
          targetCollName = this->newestContactsMsg.contact(i).collision2();
        }
        tmpLinkName = targetCollName.substr(0,
                      targetCollName.rfind("::"));
        contactLinkNames.insert(tmpLinkName);
        linkNameIndices[tmpLinkName] = i;
      }
    }

    // Iterate through the list of allowed links
    std::vector<std::string>::iterator iterLinkName =
      this->allowedLinks.begin();
    std::vector<physics::LinkPtr>::iterator iterLink = this->links.begin();
    std::vector<physics::JointPtr>::iterator iterJoint = this->joints.begin();
    unsigned int countIters = 0;
    // Only check the length of the first iterator since we used a GZ_ASSERT
    // in the Load function to confirm the vectors have the same length
    while (iterLinkName != this->allowedLinks.end())
    {
      // If *iterLinkName is in contactLinkNames
      if (contactLinkNames.end() != contactLinkNames.find(*iterLinkName))
      {
        // Compute the average contact point position
        math::Vector3 contactPositionAverage;
        {
          // Find the index to the correct contact data structure
          unsigned int i = linkNameIndices[*iterLinkName];
          if (i < nc)
          {
            unsigned int pc =
              this->newestContactsMsg.contact(i).position_size();
            // Add up all the contact point positions
            for (unsigned int j = 0; j < pc; ++j)
            {
              contactPositionAverage +=
                msgs::Convert(this->newestContactsMsg.contact(i).position(j));
            }
            // Then divide by numer of contact points
            contactPositionAverage /= static_cast<double>(pc);
          }
          else
          {
            gzerr << "Error in linkNameIndices\n";
          }
        }

        // If joint exists
        if (*iterJoint)
        {
          // Update the anchor position
          // TODO: consider checking MaxStepSize and updating erp, cfm
          (*iterJoint)->SetAnchor(0, contactPositionAverage);
        }
        // Otherwise, try to create a joint
        else
        {
          // Try to get link pointer if we don't already have it
          if (!(*iterLink))
          {
            std::string targetModelName = (*iterLinkName).substr(0,
                                          (*iterLinkName).rfind("::"));
            physics::ModelPtr targetModel =
              this->world->GetModel(targetModelName);
            if (targetModel)
              *iterLink = targetModel->GetLink(*iterLinkName);
          }

          if (*iterLink)
          {
            // Create the joint
            // gzdbg << "Creating a mud joint with " << *iterLinkName << '\n';
            (*iterLink)->SetAutoDisable(false);
            *iterJoint = this->physics->CreateJoint("revolute", this->model);

            (*iterJoint)->Attach(this->link, *iterLink);

            (*iterJoint)->Load(this->link, *iterLink,
              math::Pose(contactPositionAverage, math::Quaternion()));
            // Joint names must be unique
            // name as mud_joint_0, mud_joint_1, etc.
            {
              std::stringstream jointNameStream;
              jointNameStream << "mud_joint_" << countIters;
              (*iterJoint)->SetName(jointNameStream.str());
            }

            {
              double erp, cfm;
              erp = this->stiffness*dt / (this->stiffness*dt + this->damping);
              cfm = 1.0 / (this->stiffness*dt + this->damping);
              (*iterJoint)->SetParam("erp", 0, erp);
              (*iterJoint)->SetParam("cfm", 0, cfm);
              (*iterJoint)->SetParam("stop_erp", 0, erp);
              (*iterJoint)->SetParam("stop_cfm", 0, cfm);
            }
            (*iterJoint)->SetHighStop(0, 0.0);
            (*iterJoint)->SetLowStop(0, 0.0);

            (*iterJoint)->Init();
          }
        }
      }
      // *iterLinkName is not in contactLinkNames
      else
      {
        // If there's an existing joint,
        // then delete the joint
        if (*iterJoint)
        {
          // gzdbg << "Destroying mud joint\n";

          // reenable collision between the link pair
          physics::LinkPtr parent = (*iterJoint)->GetParent();
          physics::LinkPtr child = (*iterJoint)->GetChild();
          if (parent)
            parent->SetCollideMode("all");
          if (child)
            child->SetCollideMode("all");

          (*iterJoint)->Detach();
          (*iterJoint).reset();
        }
      }

      // Increment
      ++countIters;
      ++iterJoint;
      ++iterLink;
      ++iterLinkName;
    }

    this->newMsg = false;
    this->newMsgWait = 0;
  }
  else if (++this->newMsgWait > floor(1.0 / dt))
  {
    gzlog << "MudPlugin attached to " << this->modelName
          << " waited 1.0 s without contact messages\n";
    this->newMsgWait = 0;
  }
}
