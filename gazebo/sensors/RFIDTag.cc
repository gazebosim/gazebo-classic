/* Copyright (C)
 *     Jonas Mellin & Zakiruz Zaman
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
/* Desc: RFID Tag
 * Author: Jonas Mellin & Zakiruz Zaman 
 * Date: 6th December 2011
 */

#include "common/Exception.hh"

#include "transport/Node.hh"
#include "transport/Publisher.hh"
#include "msgs/msgs.hh"

#include "math/Vector3.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/RFIDTagManager.hh"
#include "sensors/RFIDTag.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("rfidtag", RFIDTag)

/////////////////////////////////////////////////
RFIDTag::RFIDTag()
: Sensor()
{
  this->active = false;
  this->node = transport::NodePtr(new transport::Node());
}

/////////////////////////////////////////////////
RFIDTag::~RFIDTag()
{
}

/////////////////////////////////////////////////
void RFIDTag::Load(const std::string &_worldName, sdf::ElementPtr &_sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void RFIDTag::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  if (this->sdf->GetElement("topic"))
  {
    this->node->Init(this->world->GetName());
    this->scanPub = this->node->Advertise<msgs::Pose>(
        this->sdf->GetElement("topic")->GetValueString());
  }

  this->entity = this->world->GetEntity(this->parentName);

  RFIDTagManager::Instance()->AddTaggedModel(this);
}

/////////////////////////////////////////////////
void RFIDTag::Fini()
{
  Sensor::Fini();
  this->entity.reset();
}

//////////////////////////////////////////////////
void RFIDTag::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
void RFIDTag::UpdateImpl(bool /*_force*/)
{
  if (this->scanPub)
  {
    msgs::Pose msg;
    msgs::Set(&msg, entity->GetWorldPose());

    // msg.set_position(link->GetWorldPose().pos);
    // msg.set_orientation(link->GetWorldPose().rot);
    // msgs::LaserScan msg;

    // msg.set_frame(this->link->GetScopedName());
    // msgs::Set(msg.mutable_offset(), this->GetPose());
    // msg.set_angle_min( this->GetAngleMin().Radian() );
    // msg.set_angle_max( this->GetAngleMax().Radian() );
    // msg.set_angle_step( this->GetAngleResolution() );

    // msg.set_range_min( this->GetRangeMin() );
    // msg.set_range_max( this->GetRangeMax() );

    // for (unsigned int i=0; i < (unsigned int)this->GetRangeCount(); i++)
    // {
    //   msg.add_ranges(this->laserShape->GetRange(i));
    //   msg.add_intensities(0);
    // }

    this->scanPub->Publish(msg);
    // std::cout << "update impl for rfidtag called" << std::endl;
  }
}
