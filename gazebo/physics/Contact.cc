/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/* Desc: Specification of a contact
 * Author: Nate Koenig
 * Date: 10 Nov 2009
 */

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Contact.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
Contact::Contact()
{
  this->count = 0;
}

//////////////////////////////////////////////////
Contact::Contact(const Contact &_c)
{
  *this = _c;
}

//////////////////////////////////////////////////
Contact::~Contact()
{
}

//////////////////////////////////////////////////
Contact &Contact::operator =(const Contact &_contact)
{
  this->world = _contact.world;
  this->collision1 = _contact.collision1;
  this->collision2 = _contact.collision2;

  this->count = _contact.count;
  for (int i = 0; i < MAX_CONTACT_JOINTS; i++)
  {
    this->wrench[i] = _contact.wrench[i];
    this->positions[i] = _contact.positions[i];
    this->normals[i] = _contact.normals[i];
    this->depths[i] = _contact.depths[i];
  }

  this->time = _contact.time;

  return *this;
}

//////////////////////////////////////////////////
Contact &Contact::operator =(const msgs::Contact &_contact)
{
  this->count = 0;

  this->world = physics::get_world(_contact.world());

  if (world)
  {
    this->collision1 = boost::dynamic_pointer_cast<Collision>(
        this->world->GetEntity(_contact.collision1())).get();
    this->collision2 = boost::dynamic_pointer_cast<Collision>(
      this->world->GetEntity(_contact.collision2())).get();
  }
  else
  {
    gzwarn << "World: " << _contact.world() << " not found,"
           << "contact collision pointers will be NULL";
  }

  for (int j = 0; j < _contact.position_size(); ++j)
  {
    this->positions[j] = msgs::ConvertIgn(_contact.position(j));

    this->normals[j] = msgs::ConvertIgn(_contact.normal(j));

    this->depths[j] = _contact.depth(j);

    this->wrench[j].body1Force =
      msgs::ConvertIgn(_contact.wrench(j).body_1_wrench().force());

    this->wrench[j].body2Force =
      msgs::ConvertIgn(_contact.wrench(j).body_2_wrench().force());

    this->wrench[j].body1Torque =
      msgs::ConvertIgn(_contact.wrench(j).body_1_wrench().torque());

    this->wrench[j].body2Torque =
      msgs::ConvertIgn(_contact.wrench(j).body_2_wrench().torque());

    this->count++;
  }

  this->time = msgs::Convert(_contact.time());

  return *this;
}

//////////////////////////////////////////////////
void Contact::Reset()
{
  this->count = 0;
}

//////////////////////////////////////////////////
std::string Contact::DebugString() const
{
  std::ostringstream stream;

  stream << "World [" << this->world->GetName() << "]\n"
         << "Collision 1[" << this->collision1->GetScopedName() << "]\n"
         << "Collision 2[" << this->collision2->GetScopedName() << "]\n"
         << "Time[" << this->time << "]\n"
         << "Contact Count[" << this->count << "]\n";

  for (int i = 0; i < this->count; ++i)
  {
    stream << "--- Contact[" << i << "]\n";
    stream << "  Depth[" << this->depths[i] << "]\n"
           << "  Position[" << this->positions[i] << "]\n"
           << "  Normal[" << this->normals[i] << "]\n"
           << "  Force1[" << this->wrench[i].body1Force << "]\n"
           << "  Force2[" << this->wrench[i].body2Force << "]\n"
           << "  Torque1[" << this->wrench[i].body1Torque << "]\n"
           << "  Torque2[" << this->wrench[i].body2Torque << "]\n";
  }

  return stream.str();
}

//////////////////////////////////////////////////
void Contact::FillMsg(msgs::Contact &_msg) const
{
  _msg.set_world(this->world->GetName());
  _msg.set_collision1(this->collision1->GetScopedName());
  _msg.set_collision2(this->collision2->GetScopedName());
  msgs::Set(_msg.mutable_time(), this->time);

  for (int j = 0; j < this->count; ++j)
  {
    _msg.add_depth(this->depths[j]);

    msgs::Set(_msg.add_position(), this->positions[j].Ign());
    msgs::Set(_msg.add_normal(), this->normals[j].Ign());

    msgs::JointWrench *jntWrench = _msg.add_wrench();
    jntWrench->set_body_1_name(this->collision1->GetScopedName());
    jntWrench->set_body_1_id(this->collision1->GetId());
    jntWrench->set_body_2_name(this->collision2->GetScopedName());
    jntWrench->set_body_2_id(this->collision2->GetId());

    msgs::Wrench *wrenchMsg =  jntWrench->mutable_body_1_wrench();
    msgs::Set(wrenchMsg->mutable_force(), this->wrench[j].body1Force.Ign());
    msgs::Set(wrenchMsg->mutable_torque(), this->wrench[j].body1Torque.Ign());

    wrenchMsg =  jntWrench->mutable_body_2_wrench();
    msgs::Set(wrenchMsg->mutable_force(), this->wrench[j].body2Force.Ign());
    msgs::Set(wrenchMsg->mutable_torque(), this->wrench[j].body2Torque.Ign());
  }
}
