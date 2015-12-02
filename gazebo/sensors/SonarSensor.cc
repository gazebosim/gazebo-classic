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

#include <boost/algorithm/string.hpp>

#include <ignition/math/Vector3.hh>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/MeshShape.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/Collision.hh"

#include "gazebo/common/Assert.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SonarSensorPrivate.hh"
#include "gazebo/sensors/SonarSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("sonar", SonarSensor)

//////////////////////////////////////////////////
SonarSensor::SonarSensor()
: Sensor(*new SonarSensorPrivate, sensors::OTHER),
  dataPtr(std::static_pointer_cast<SonarSensorPrivate>(this->dPtr))
{
  this->dataPtr->emptyContactCount = 0;
}

//////////////////////////////////////////////////
SonarSensor::~SonarSensor()
{
  this->dataPtr->sonarCollision->Fini();
  this->dataPtr->sonarCollision.reset();

  this->dataPtr->sonarShape->Fini();
  this->dataPtr->sonarShape.reset();
}

//////////////////////////////////////////////////
std::string SonarSensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/sonar";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void SonarSensor::Load(const std::string &_worldName)
{
  sdf::ElementPtr sonarElem = this->dataPtr->sdf->GetElement("sonar");
  if (!sonarElem)
  {
    gzerr << "Sonar sensor is missing <sonar> SDF element";
    return;
  }

  this->dataPtr->rangeMin = sonarElem->Get<double>("min");
  this->dataPtr->rangeMax = sonarElem->Get<double>("max");
  this->dataPtr->radius = sonarElem->Get<double>("radius");
  double range = this->dataPtr->rangeMax - this->dataPtr->rangeMin;

  if (this->dataPtr->radius < 0)
  {
    gzerr << "Sonar radius must be > 0. Current value is["
      << this->dataPtr->radius << "]\n";
    return;
  }

  if (this->dataPtr->rangeMin < 0)
  {
    gzerr << "Min sonar range must be >= 0. Current value is["
      << this->dataPtr->rangeMin << "]\n";
    return;
  }

  if (this->dataPtr->rangeMin > this->dataPtr->rangeMax)
  {
    gzerr << "Min sonar range of [" << this->dataPtr->rangeMin
          << "] must be less than" << "the max sonar range of["
          << this->dataPtr->rangeMax << "]\n";
    return;
  }

  Sensor::Load(_worldName);
  GZ_ASSERT(this->dataPtr->world != NULL,
      "SonarSensor did not get a valid World pointer");

  this->dataPtr->parentEntity =
    this->dataPtr->world->GetEntity(this->ParentName());

  GZ_ASSERT(this->dataPtr->parentEntity != NULL,
      "Unable to get the parent entity.");

  physics::PhysicsEnginePtr physicsEngine =
    this->dataPtr->world->GetPhysicsEngine();

  GZ_ASSERT(physicsEngine != NULL,
      "Unable to get a pointer to the physics engine");

  /// \todo: Change the collision shape to a cone. Needs a collision shape
  /// within ODE. Or, switch out the collision engine.
  this->dataPtr->sonarCollision = physicsEngine->CreateCollision("mesh",
      this->ParentName());

  GZ_ASSERT(this->dataPtr->sonarCollision != NULL,
      "Unable to create a cylinder collision using the physics engine.");

  this->dataPtr->sonarCollision->SetName(this->ScopedName() +
      "sensor_collision");

  // We need a few contacts in order to get the closest collision. This is
  // not guaranteed to return the closest contact.
  this->dataPtr->sonarCollision->SetMaxContacts(2);
  this->dataPtr->sonarCollision->AddType(physics::Base::SENSOR_COLLISION);
  this->dataPtr->parentEntity->AddChild(this->dataPtr->sonarCollision);

  this->dataPtr->sonarShape = boost::dynamic_pointer_cast<physics::MeshShape>(
      this->dataPtr->sonarCollision->GetShape());

  GZ_ASSERT(this->dataPtr->sonarShape != NULL,
      "Unable to get the sonar shape from the sonar collision.");

  // Use a scaled cone mesh for the sonar collision shape.
  this->dataPtr->sonarShape->SetMesh("unit_cone");
  this->dataPtr->sonarShape->SetScale(ignition::math::Vector3d(
        this->dataPtr->radius*2.0, this->dataPtr->radius*2.0, range));

  // Position the collision shape properly. Without this, the shape will be
  // centered at the start of the sonar.
  ignition::math::Vector3d offset(0, 0, range * 0.5);
  offset = this->dataPtr->pose.Rot().RotateVector(offset);
  this->dataPtr->sonarMidPose.Set(this->dataPtr->pose.Pos() - offset,
      this->dataPtr->pose.Rot());

  this->dataPtr->sonarCollision->SetRelativePose(this->dataPtr->sonarMidPose);
  this->dataPtr->sonarCollision->SetInitialRelativePose(
      this->dataPtr->sonarMidPose);

  // Don't create contacts when objects collide with the sonar shape
  this->dataPtr->sonarCollision->GetSurface()->collideWithoutContact = true;
  this->dataPtr->sonarCollision->GetSurface()->collideWithoutContactBitmask = 1;
  this->dataPtr->sonarCollision->SetCollideBits(~GZ_SENSOR_COLLIDE);
  this->dataPtr->sonarCollision->SetCategoryBits(GZ_SENSOR_COLLIDE);

  /*std::vector<std::string> collisions;
  collisions.push_back(this->dataPtr->sonarCollision->GetScopedName());

  physics::ContactManager *contactMgr =
    this->dataPtr->world->GetPhysicsEngine()->GetContactManager();
    */

  // Create a contact topic for the collision shape
  std::string topic =
    this->dataPtr->world->GetPhysicsEngine()->GetContactManager()->CreateFilter(
        this->dataPtr->sonarCollision->GetScopedName(),
        this->dataPtr->sonarCollision->GetScopedName());

  // Subscribe to the contact topic
  this->dataPtr->contactSub = this->dataPtr->node->Subscribe(topic,
      &SonarSensor::OnContacts, this);

  // Advertise the sensor's topic on which we will output range data.
  this->dataPtr->sonarPub = this->dataPtr->node->Advertise<msgs::SonarStamped>(
      this->Topic());

  // Initialize the message that will be published on this->dataPtr->sonarPub.
  this->dataPtr->sonarMsg.mutable_sonar()->set_range_min(
      this->dataPtr->rangeMin);
  this->dataPtr->sonarMsg.mutable_sonar()->set_range_max(
      this->dataPtr->rangeMax);
  this->dataPtr->sonarMsg.mutable_sonar()->set_radius(
      this->dataPtr->radius);

  msgs::Set(this->dataPtr->sonarMsg.mutable_sonar()->mutable_world_pose(),
      this->dataPtr->sonarMidPose);
  this->dataPtr->sonarMsg.mutable_sonar()->set_range(0);
}

//////////////////////////////////////////////////
void SonarSensor::Init()
{
  Sensor::Init();
  this->dataPtr->sonarMsg.mutable_sonar()->set_frame(this->ParentName());
  msgs::Set(this->dataPtr->sonarMsg.mutable_time(),
      this->dataPtr->world->GetSimTime());
  this->dataPtr->sonarMsg.mutable_sonar()->set_range(this->dataPtr->rangeMax);

  if (this->dataPtr->sonarPub)
    this->dataPtr->sonarPub->Publish(this->dataPtr->sonarMsg);
}

//////////////////////////////////////////////////
void SonarSensor::Fini()
{
  if (this->dataPtr->world && this->dataPtr->world->GetRunning())
  {
    physics::ContactManager *mgr =
        this->dataPtr->world->GetPhysicsEngine()->GetContactManager();
    mgr->RemoveFilter(this->dataPtr->sonarCollision->GetScopedName());
  }

  this->dataPtr->sonarPub.reset();
  this->dataPtr->contactSub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
double SonarSensor::GetRangeMin() const
{
  return this->RangeMin();
}

//////////////////////////////////////////////////
double SonarSensor::RangeMin() const
{
  return this->dataPtr->rangeMin;
}

//////////////////////////////////////////////////
double SonarSensor::GetRangeMax() const
{
  return this->RangeMax();
}

//////////////////////////////////////////////////
double SonarSensor::RangeMax() const
{
  return this->dataPtr->rangeMax;
}

//////////////////////////////////////////////////
double SonarSensor::GetRadius() const
{
  return this->Radius();
}

//////////////////////////////////////////////////
double SonarSensor::Radius() const
{
  return this->dataPtr->radius;
}

//////////////////////////////////////////////////
double SonarSensor::GetRange()
{
  return this->Range();
}

//////////////////////////////////////////////////
double SonarSensor::Range()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->sonarMsg.sonar().range();
}

//////////////////////////////////////////////////
bool SonarSensor::UpdateImpl(const bool /*_force*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->lastMeasurementTime = this->dataPtr->world->GetSimTime();
  msgs::Set(this->dataPtr->sonarMsg.mutable_time(),
            this->dataPtr->lastMeasurementTime);

  ignition::math::Pose3d referencePose =
    this->dataPtr->pose + this->dataPtr->parentEntity->GetWorldPose().Ign();
  ignition::math::Vector3d pos;

  // A 5-step hysteresis window was chosen to reduce range value from
  // bouncing.
  if (!this->dataPtr->incomingContacts.empty() ||
      this->dataPtr->emptyContactCount > 5)
  {
    this->dataPtr->sonarMsg.mutable_sonar()->set_range(
        this->dataPtr->rangeMax);
    this->dataPtr->emptyContactCount = 0;
  }
  else
  {
    ++this->dataPtr->emptyContactCount;
  }


  // Iterate over all the contact messages
  for (auto iter = this->dataPtr->incomingContacts.begin();
      iter != this->dataPtr->incomingContacts.end(); ++iter)
  {
    // Iterate over all the contacts in the message
    for (int i = 0; i < (*iter)->contact_size(); ++i)
    {
      // Debug output:
      // std::cout << "C1[" << (*iter)->contact(i).collision1() << "]"
      //   << "C2[" << (*iter)->contact(i).collision2() << "]\n";

      for (int j = 0; j < (*iter)->contact(i).position_size(); ++j)
      {
        // Get the contact position relative to the reference position.
        pos = msgs::ConvertIgn((*iter)->contact(i).position(j)) -
          referencePose.Pos();

        // Compute the sensed range.
        double len = pos.Length() - (*iter)->contact(i).depth(j);

        // Debug output:
        // std::cout << "  RP[" << referencePose << "]  P[" << pos
        //   << "] L[" << len << "] D["
        //   << (*iter)->contact(i).depth(j) << "]\n";

        // Copy the contact message.
        if (len < this->dataPtr->sonarMsg.sonar().range())
        {
          this->dataPtr->sonarMsg.mutable_sonar()->set_range(len);
        }
      }
    }
  }

  // Clear the incoming contact list.
  this->dataPtr->incomingContacts.clear();

  this->dataPtr->update(this->dataPtr->sonarMsg);

  if (this->dataPtr->sonarPub)
    this->dataPtr->sonarPub->Publish(this->dataPtr->sonarMsg);

  return true;
}

//////////////////////////////////////////////////
bool SonarSensor::IsActive()
{
  return Sensor::IsActive() || this->dataPtr->sonarPub->HasConnections();
}

//////////////////////////////////////////////////
void SonarSensor::OnContacts(ConstContactsPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Only store information if the sensor is active
  if (this->IsActive() && _msg->contact_size() > 0)
  {
    // Store the contacts message for processing in UpdateImpl
    this->dataPtr->incomingContacts.push_back(_msg);

    // Prevent the incomingContacts list to grow indefinitely.
    if (this->dataPtr->incomingContacts.size() > 100)
      this->dataPtr->incomingContacts.pop_front();
  }
}

//////////////////////////////////////////////////
event::ConnectionPtr SonarSensor::ConnectUpdate(
    std::function<void (msgs::SonarStamped)> _subscriber)
{
  return this->dataPtr->update.Connect(_subscriber);
}

//////////////////////////////////////////////////
void SonarSensor::DisconnectUpdate(event::ConnectionPtr &_conn)
{
  this->dataPtr->update.Disconnect(_conn);
}

