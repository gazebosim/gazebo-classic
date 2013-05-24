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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ContactManager.hh"
// #include "gazebo/physics/Physics.hh"
// #include "gazebo/physics/Model.hh"
#include "gazebo/physics/Collision.hh"

#include "gazebo/common/Assert.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SonarSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("sonar", SonarSensor)

//////////////////////////////////////////////////
SonarSensor::SonarSensor()
: Sensor(sensors::OTHER)
{
}

//////////////////////////////////////////////////
SonarSensor::~SonarSensor()
{
  this->sonarCollision->Fini();
  this->sonarCollision.reset();

  this->sonarShape->Fini();
  this->sonarShape.reset();
}

//////////////////////////////////////////////////
std::string SonarSensor::GetTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/sonar";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void SonarSensor::Load(const std::string &_worldName)
{
  sdf::ElementPtr sonarElem = this->sdf->GetElement("sonar");
  if (!sonarElem)
  {
    gzerr << "Sonar sensor is missing <sonar> SDF element";
    return;
  }

  double min = sonarElem->GetValueDouble("min");
  double max = sonarElem->GetValueDouble("max");
  double radius = sonarElem->GetValueDouble("radius");

  if (radius < 0)
  {
    gzerr << "Sonar radius must be > 0. Current value is[" << radius << "]\n";
    return;
  }

  if (min < 0)
  {
    gzerr << "Min sonar range must be >= 0. Current value is[" << min << "]\n";
    return;
  }

  if (min > max)
  {
    gzerr << "Min sonar range of [" << min << "] must be less than"
      << "the max sonar range of[" << max << "]\n";
    return;
  }

  if (max < min)
  {
    gzerr << "Max sonar range of [" << max << "] must be greater than"
      << "the min sonar range of[" << min << "]\n";
    return;
  }

  Sensor::Load(_worldName);
  GZ_ASSERT(this->world != NULL,
      "SonarSensor did not get a valid World pointer");

  this->parentEntity = this->world->GetEntity(this->parentName);

  GZ_ASSERT(this->parentEntity != NULL,
      "Unable to get the parent entity.");

  physics::PhysicsEnginePtr physicsEngine = this->world->GetPhysicsEngine();

  GZ_ASSERT(physicsEngine != NULL,
      "Unable to get a pointer to the physics engine");

  this->sonarCollision = physicsEngine->CreateCollision("cylinder",
      this->parentName);

  GZ_ASSERT(this->sonarCollision != NULL,
      "Unable to create a cylinder collision using the physics engine.");

  this->sonarCollision->SetName(this->GetScopedName() + "sensor_collision");
  this->sonarCollision->AddType(physics::Base::SENSOR_COLLISION);
  this->parentEntity->AddChild(this->sonarCollision);

  this->sonarShape = boost::dynamic_pointer_cast<physics::CylinderShape>(
      this->sonarCollision->GetShape());

  GZ_ASSERT(this->sonarShape != NULL,
      "Unable to get the sonar shape from the sonar collision.");

  this->sonarMsg.mutable_sonar()->set_range_min(min);
  this->sonarMsg.mutable_sonar()->set_range_max(max);
  this->sonarMsg.mutable_sonar()->set_radius(radius);

  this->sonarShape->SetRadius(radius);
  this->sonarShape->SetLength(max - min);

  math::Vector3 offset(0, 0, (max - min) * 0.5);
  offset = this->pose.rot.RotateVector(offset);

  this->sonarMidPose.Set(this->pose.pos - offset, this->pose.rot);

  std::vector<std::string> collisions;
  collisions.push_back(this->sonarCollision->GetScopedName());

  physics::ContactManager *contactMgr =
    this->world->GetPhysicsEngine()->GetContactManager();

  std::string topic = contactMgr->CreateFilter(
      this->sonarCollision->GetScopedName(), collisions);

  if (!this->contactSub)
  {
    this->contactSub = this->node->Subscribe(topic,
        &SonarSensor::OnContacts, this);
  }

  this->sonarCollision->SetRelativePose(this->sonarMidPose);
  this->sonarCollision->SetInitialRelativePose(this->sonarMidPose);

  this->sonarCollision->GetSurface()->collideWithoutContact = true;
  this->sonarCollision->GetSurface()->collideWithoutContactBitmask = 1;
  this->sonarCollision->SetCollideBits(~GZ_SENSOR_COLLIDE);
  this->sonarCollision->SetCategoryBits(GZ_SENSOR_COLLIDE);

  msgs::Set(this->sonarMsg.mutable_sonar()->mutable_world_pose(),
      this->sonarMidPose);
  this->sonarMsg.mutable_sonar()->set_range(0);

  this->sonarPub = this->node->Advertise<msgs::SonarStamped>(this->GetTopic());
}

//////////////////////////////////////////////////
void SonarSensor::Init()
{
  Sensor::Init();
  this->sonarMsg.mutable_sonar()->set_frame(this->parentName);
  msgs::Set(this->sonarMsg.mutable_time(), this->world->GetSimTime());

  if (this->sonarPub)
    this->sonarPub->Publish(this->sonarMsg);
}

//////////////////////////////////////////////////
void SonarSensor::Fini()
{
  this->sonarPub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
double SonarSensor::GetRange()
{
  boost::mutex::scoped_lock lock(this->mutex);
  return this->sonarMsg.sonar().range();
}

//////////////////////////////////////////////////
void SonarSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->sonarMsg.mutable_time(), this->lastMeasurementTime);
  this->sonarMsg.mutable_sonar()->set_range(0);

  math::Pose referencePose = this->pose + this->parentEntity->GetWorldPose();
  math::Vector3 pos;

  // Iterate over all the contact messages
  for (ContactMsgs_L::iterator iter = this->incomingContacts.begin();
      iter != this->incomingContacts.end(); ++iter)
  {
    // Iterate over all the contacts in the message
    for (int i = 0; i < (*iter)->contact_size(); ++i)
    {
      // Debug output:
      // std::cout << "Collision1[" << (*iter)->contact(i).collision1() << "]"
      //  << "C2[" << (*iter)->contact(i).collision2() << "]\n";
      
      this->sonarMsg.mutable_sonar()->set_range(-1);

      for (int j = 0; j < (*iter)->contact(i).position_size(); ++j)
      {
        pos = msgs::Convert((*iter)->contact(i).position(j));
        math::Vector3 relPos = pos - referencePose.pos;
        double len = pos.Distance(referencePose.pos);

        // Debug output:
        // std::cout << "  SP[" << this->pose.pos << "]  P[" << pos
        //  << "] Len[" << len << "]D["
        //  << (*iter)->contact(i).depth(j) << "]\n";

        // Copy the contact message.
        if (this->sonarMsg.sonar().range() < 0 ||
            len < this->sonarMsg.sonar().range())
        {
          this->sonarMsg.mutable_sonar()->set_range(len);
        }
      }
    }
  }

  // Clear the incoming contact list.
  this->incomingContacts.clear();

  if (this->sonarPub)
    this->sonarPub->Publish(this->sonarMsg);
}

//////////////////////////////////////////////////
bool SonarSensor::IsActive()
{
  return Sensor::IsActive() || this->sonarPub->HasConnections();
}

//////////////////////////////////////////////////
void SonarSensor::OnContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Only store information if the sensor is active
  if (this->IsActive() && _msg->contact_size() > 0)
  {
    // Store the contacts message for processing in UpdateImpl
    this->incomingContacts.push_back(_msg);

    // Prevent the incomingContacts list to grow indefinitely.
    if (this->incomingContacts.size() > 100)
      this->incomingContacts.pop_front();
  }
}
