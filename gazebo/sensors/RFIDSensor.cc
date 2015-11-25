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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"

#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/math/Vector3.hh"

#include "gazebo/sensors/RFIDTag.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/RFIDSensorPrivate.hh"
#include "gazebo/sensors/RFIDSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("rfid", RFIDSensor)

/////////////////////////////////////////////////
RFIDSensor::RFIDSensor()
  : Sensor(*new RFIDSensorPrivate, sensors::OTHER)
{
  this->dataPtr = std::static_pointer_cast<RFIDSensorPrivate>(this->dPtr);
  this->dataPtr->active = false;
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

  if (this->dataPtr->sdf->GetElement("topic"))
  {
    this->dataPtr->scanPub = this->dataPtr->node->Advertise<msgs::Pose>(
        this->dataPtr->sdf->GetElement("topic")->Get<std::string>());
  }

  this->dataPtr->entity = this->dataPtr->world->GetEntity(this->ParentName());

  // this->dataPtr->sdf->PrintDescription("something");
  /*std::cout << " setup ray" << std::endl;
  physics::PhysicsEnginePtr physicsEngine = world->GetPhysicsEngine();

  //trying to use just "ray" gives a seg fault
  this->laserCollision = physicsEngine->CreateCollision("multiray",
      this->parentName);

  this->laserCollision->SetName("rfid_sensor_collision");
  this->laserCollision->SetRelativePose(this->pose);

  this->laserShape = boost::dynamic_pointer_cast<physics::RayShape>(
      this->laserCollision->GetShape());

  this->laserShape->Load(this->dataPtr->sdf);

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
bool RFIDSensor::UpdateImpl(const bool /*_force*/)
{
  this->EvaluateTags();
  this->dataPtr->lastMeasurementTime = this->dataPtr->world->GetSimTime();

  if (this->dataPtr->scanPub)
  {
    msgs::Pose msg;
    msgs::Set(&msg, this->dataPtr->entity->GetWorldPose().Ign());
    this->dataPtr->scanPub->Publish(msg);
  }

  return true;
}

//////////////////////////////////////////////////
void RFIDSensor::EvaluateTags()
{
  std::vector<RFIDTag*>::const_iterator ci;

  // iterate through the tags contained given rfid tag manager
  for (ci = this->dataPtr->tags.begin(); ci != this->dataPtr->tags.end(); ++ci)
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
bool RFIDSensor::CheckTagRange(const ignition::math::Pose3d &_pose)
{
  // copy sensor vector pos into a temp var
  ignition::math::Vector3d v;
  v = _pose.Pos() - this->dataPtr->entity->GetWorldPose().Ign().Pos();

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
  this->dataPtr->tags.push_back(_tag);
}
