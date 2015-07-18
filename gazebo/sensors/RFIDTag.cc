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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/RFIDSensor.hh"
#include "gazebo/sensors/RFIDTag.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("rfidtag", RFIDTag)

/////////////////////////////////////////////////
RFIDTag::RFIDTag()
: Sensor(sensors::OTHER)
{
  this->active = false;
}

/////////////////////////////////////////////////
RFIDTag::~RFIDTag()
{
}

/////////////////////////////////////////////////
void RFIDTag::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void RFIDTag::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  if (this->sdf->GetElement("topic"))
  {
    this->scanPub = this->node->Advertise<msgs::Pose>(
        this->sdf->GetElement("topic")->Get<std::string>());
  }

  this->entity = this->world->GetEntity(this->parentName);

  // Add the tag to all the RFID sensors.
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter)
  {
    if ((*iter)->GetType() == "rfid")
    {
      boost::dynamic_pointer_cast<RFIDSensor>(*iter)->AddTag(this);
    }
  }
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
bool RFIDTag::UpdateImpl(bool /*_force*/)
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

    // for (unsigned int i = 0; i < (unsigned int)this->GetRangeCount(); i++)
    // {
    //   msg.add_ranges(this->laserShape->GetRange(i));
    //   msg.add_intensities(0);
    // }

    this->scanPub->Publish(msg);
    // std::cout << "update impl for rfidtag called" << std::endl;
  }

  return true;
}

/////////////////////////////////////////////////
math::Pose RFIDTag::GetTagPose() const
{
  return this->TagPose();
}

/////////////////////////////////////////////////
ignition::math::Pose3d RFIDTag::TagPose() const
{
  return entity->GetWorldPose().Ign();
}
