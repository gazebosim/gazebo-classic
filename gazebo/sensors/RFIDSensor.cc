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
/* Desc: RFID Sensor
 * Author: Jonas Mellin & Zakiruz Zaman
 * Date: 6th December 2011
 */

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"

#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/math/Vector3.hh"

#include "gazebo/sensors/RFIDTag.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/RFIDSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("rfid", RFIDSensor)

/////////////////////////////////////////////////
RFIDSensor::RFIDSensor()
  : Sensor(sensors::OTHER)
{
  this->active = false;
}

/////////////////////////////////////////////////
RFIDSensor::~RFIDSensor()
{
  // this->link.reset();
}

/////////////////////////////////////////////////
void RFIDSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf )
{
  Sensor::Load(_worldName, _sdf);
}

/////////////////////////////////////////////////
void RFIDSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  // std::cout << "load rfid sensor" << std::endl;

  if (this->sdf->GetElement("topic"))
  {
    this->scanPub = this->node->Advertise<msgs::Pose>(
        this->sdf->GetElement("topic")->Get<std::string>());
  }

  this->entity = this->world->GetEntity(this->parentName);

  // this->sdf->PrintDescription("something");
  /*std::cout << " setup ray" << std::endl;
  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  //trying to use just "ray" gives a seg fault
  this->laserCollision = physicsEngine->CreateCollision("multiray",
      this->parentName);

  this->laserCollision->SetName("rfid_sensor_collision");
  this->laserCollision->SetRelativePose(this->pose);

  this->laserShape = boost::dynamic_pointer_cast<physics::RayShape>(
      this->laserCollision->GetShape());

  this->laserShape->Load(this->sdf);

  this->laserShape->Init();
  */

  /*** Tried to use rendering, but says rendering engine isnt initialized
       which is understandable */

  /**
    rendering::ScenePtr scene = rendering::get_scene(this->world->GetName());
    if (!scene)
    {
    scene = rendering::create_scene(this->world->GetName(), false);
    }

    manager = rendering::get_scene(this->world->GetName())->GetManager();

    query = manager->createRayQuery(Ogre::Ray());

*/
}

/////////////////////////////////////////////////
void RFIDSensor::Fini()
{
  Sensor::Fini();
}

//////////////////////////////////////////////////
void RFIDSensor::Init()
{
  Sensor::Init();
}

//////////////////////////////////////////////////
bool RFIDSensor::UpdateImpl(bool /*_force*/)
{
  this->EvaluateTags();
  this->lastMeasurementTime = this->world->GetSimTime();

  if (this->scanPub)
  {
    msgs::Pose msg;
    msgs::Set(&msg, this->entity->GetWorldPose());
    this->scanPub->Publish(msg);
  }

  return true;
}

//////////////////////////////////////////////////
void RFIDSensor::EvaluateTags()
{
  std::vector<RFIDTag*>::const_iterator ci;

  // iterate through the tags contained given rfid tag manager
  for (ci = this->tags.begin(); ci != this->tags.end(); ++ci)
  {
    ignition::math::Pose3d pos = (*ci)->TagPose();
    // std::cout << "link: " << tagModelPtr->GetName() << std::endl;
    // std::cout << "link pos: x" << pos.pos.x
    //     << " y:" << pos.pos.y
    //     << " z:" << pos.pos.z << std::endl;
    this->CheckTagRange(pos);
  }
}

//////////////////////////////////////////////////
bool RFIDSensor::CheckTagRange(const math::Pose &_pose)
{
  return this->CheckTagRange(_pose.Ign());
}

//////////////////////////////////////////////////
bool RFIDSensor::CheckTagRange(const ignition::math::Pose3d &_pose)
{
  // copy sensor vector pos into a temp var
  ignition::math::Vector3d v;
  v = _pose.Pos() - this->entity->GetWorldPose().Ign().Pos();

  // std::cout << v.GetLength() << std::endl;

  if (v.Length() <= 5.0)
  {
    // std::cout << "detected " <<  v.GetLength() << std::endl;
    return true;
  }

  // this->CheckRayIntersection(link);
  return false;
}

//////////////////////////////////////////////////
void RFIDSensor::AddTag(RFIDTag *_tag)
{
  this->tags.push_back(_tag);
}
