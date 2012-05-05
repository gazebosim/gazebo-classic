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

#include "physics/physics.h"
#include "transport/transport.h"
#include "plugins/GripperPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GripperPlugin)

/////////////////////////////////////////////////
GripperPlugin::GripperPlugin()
{
}

/////////////////////////////////////////////////
void GripperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  /*this->physics = this->model->GetWorld()->GetPhysicsEngine();
  this->joint = this->physics->CreateJoint("revolute");
  this->joint->SetModel(this->model);
  this->joint->Load();
  */

  sdf::ElementPtr jointElem = _sdf->GetElement("joint");

  physics::JointPtr joint;
  physics::LinkPtr tmplinks[2];
  physics::CollisionPtr collision;
  std::map<std::string, physics::CollisionPtr>::iterator collIter;
  while (jointElem)
  {
    std::cout << "Joint[" << jointElem->GetValueString() << "]\n";
    joint = this->model->GetJoint(jointElem->GetValueString());
    tmplinks[0] = joint->GetParent();
    tmplinks[1] = joint->GetChild();

    for (unsigned int i = 0; i < 2; ++i)
    {
      for (unsigned int j = 0; j < tmplinks[i]->GetChildCount(); ++j)
      {
        collision = tmplinks[i]->GetCollision(j);
        collIter = this->collisions.find(collision->GetScopedName());
        if (collIter != this->collisions.end()) 
          continue;
        
        collision->SetContactsEnabled(true);
        std::cout << "Collision[" << collision->GetScopedName() << "]\n";
        this->connections.push_back(collision->ConnectContact(
              boost::bind(&GripperPlugin::OnContact, this, _1, _2)));
        this->collisions[collision->GetScopedName()] = collision;
      }
    }

    jointElem = jointElem->GetNextElement("joint");
  }

  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
          boost::bind(&GripperPlugin::OnUpdate, this)));
}

/////////////////////////////////////////////////
void GripperPlugin::Init()
{
}

/////////////////////////////////////////////////
void GripperPlugin::OnUpdate()
{
  if (this->contacts.size() >= 2)
  {
    std::map<std::string, int> contactCounts;
    std::map<std::string, int>::iterator iter;

    for (unsigned int i = 0; i < this->contacts.size(); ++i)
    {
      std::string name1 = this->contacts[i].collision1->GetScopedName();
      std::string name2 = this->contacts[i].collision2->GetScopedName();

      if (this->collisions.find(name1) == this->collisions.end())
        contactCounts[name1] += 1;
      if (this->collisions.find(name2) == this->collisions.end())
        contactCounts[name2] += 1;
    }

    printf("--------------------\n");
    iter = contactCounts.begin();
    while (iter != contactCounts.end())
    {
      if (iter->second < 2)
        contactCounts.erase(iter++);
      else
      {
        std::cout << iter->first << ":" << iter->second << "\n";
        ++iter;
      }
    }

  }
  this->contacts.clear();
}

/////////////////////////////////////////////////
void GripperPlugin::OnContact(const std::string &_collisionName,
                              const physics::Contact &_contact)
{
  if (_contact.collision1->IsStatic() || _contact.collision2->IsStatic())
    return;

  //std::cout << "OnContact[" << _collisionName << "]\n";
  //std::cout << "   C1[" << _contact.collision1->GetScopedName() << "]";
  //std::cout << " C2[" << _contact.collision2->GetScopedName() << "]\n";
  //std::cout << "   N[" << _contact.normals[0] << "]\n";
  //std::cout << "  Static[" << _contact.collision2->IsStatic() << "]\n";
  this->contacts.push_back(_contact);
}
