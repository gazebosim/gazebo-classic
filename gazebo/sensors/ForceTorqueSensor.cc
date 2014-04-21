/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Joint.hh"

#include "gazebo/common/Assert.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("force_torque", ForceTorqueSensor)

//////////////////////////////////////////////////
ForceTorqueSensor::ForceTorqueSensor()
: Sensor(sensors::OTHER)
{
}

//////////////////////////////////////////////////
ForceTorqueSensor::~ForceTorqueSensor()
{
}

//////////////////////////////////////////////////
std::string ForceTorqueSensor::GetTopic() const
{
  std::string topicName = "~/";
  topicName += this->parentName + "/" + this->GetName() + "/wrench";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName,
                             sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  sdf::ElementPtr forceTorqueElem = this->sdf->GetElement("force_torque");

  // Handle frame setting
  MeasureFrame default_frame = CHILD_LINK;
  if (forceTorqueElem->HasElement("frame"))
  {
    sdf::ElementPtr frameElem = forceTorqueElem->GetElement("frame");
    std::string measureFrameSDF;
    frameElem->GetValue()->Get<std::string>(measureFrameSDF);
    if (measureFrameSDF == "parent")
    {
      measure_frame = PARENT_LINK;
    }
    else if (measureFrameSDF == "child")
    {
      measure_frame = CHILD_LINK;
    }
    else if (measureFrameSDF == "joint")
    {
      measure_frame = JOINT;
    }
    else
    {
      gzwarn << "ignoring unknown force_torque frame \"" << measureFrameSDF
             << "\", using default value" << std::endl;
      measure_frame = default_frame;
    }
  }
  else
  {
    measure_frame = default_frame;
  }

  // Save joint_child orientation, useful if the measure
  // is expressed in joint orientation
  GZ_ASSERT(this->parentJoint,
            "parentJoint should be defined by single argument Load()");
  math::Quaternion rotation_child_joint =
    this->parentJoint->GetInitialAnchorPose().rot;
  this->rotation_joint_child =
    rotation_child_joint.GetInverse().GetAsMatrix3();

  // Handle measure direction
  bool default_direction_is_parent_to_child = false;
  if (forceTorqueElem->HasElement("measure_direction"))
  {
    sdf::ElementPtr measureDirectionElem =
      forceTorqueElem->GetElement("measure_direction");
    std::string measureDirectionSDF;
    measureDirectionElem->GetValue()->Get<std::string>(measureDirectionSDF);
    if (measureDirectionSDF == "parent_to_child")
    {
      parent_to_child = true;
    }
    else if (measureDirectionSDF == "child_to_parent")
    {
      parent_to_child = false;
    }
    else
    {
      gzwarn << "ignoring unknown force_torque measure_direction \""
             << measureDirectionSDF << "\", using default value" << std::endl;
      parent_to_child = default_direction_is_parent_to_child;
    }
  }
  else
  {
    parent_to_child = default_direction_is_parent_to_child;
  }
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  GZ_ASSERT(this->world != NULL,
      "ForceTorqueSensor did not get a valid World pointer");

  this->parentJoint = boost::dynamic_pointer_cast<physics::Joint>(
      this->world->GetByName(this->parentName));

  if (!this->parentJoint)
  {
    gzerr << "Parent of a force torque sensor[" << this->GetName()
          << "] Must be a joint\n";
    return;
  }

  this->wrenchPub = this->node->Advertise<msgs::WrenchStamped>(
      this->GetTopic());
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Init()
{
  Sensor::Init();
  this->parentJoint->SetProvideFeedback(true);
}

//////////////////////////////////////////////////
void ForceTorqueSensor::Fini()
{
  this->wrenchPub.reset();
  Sensor::Fini();
}

//////////////////////////////////////////////////
physics::JointPtr ForceTorqueSensor::GetJoint() const
{
  return this->parentJoint;
}

//////////////////////////////////////////////////
math::Vector3 ForceTorqueSensor::GetForce() const
{
  return msgs::Convert(this->wrenchMsg.wrench().force());
}

//////////////////////////////////////////////////
math::Vector3 ForceTorqueSensor::GetTorque() const
{
  return msgs::Convert(this->wrenchMsg.wrench().torque());
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::UpdateImpl(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->lastMeasurementTime = this->world->GetSimTime();
  msgs::Set(this->wrenchMsg.mutable_time(), this->lastMeasurementTime);

  physics::JointWrench wrench = this->parentJoint->GetForceTorque(0u);

  // Get the force and torque in the parent frame.
  /*
  msgs::Set(this->wrenchMsg.mutable_wrench()->mutable_force(),
      wrench.body2Force);
  msgs::Set(this->wrenchMsg.mutable_wrench()->mutable_torque(),
      wrench.body2Torque);
  */
  math::Vector3 measured_force;
  math::Vector3 measured_torque;

  if (measure_frame == PARENT_LINK)
  {
    if (parent_to_child)
    {
      measured_force = wrench.body1Force;
      measured_torque = wrench.body1Torque;
    }
    else
    {
      measured_force = -1*wrench.body1Force;
      measured_torque = -1*wrench.body1Torque;
    }
  }
  else if (measure_frame == CHILD_LINK)
  {
    if (!parent_to_child)
    {
      measured_force = wrench.body2Force;
      measured_torque = wrench.body2Torque;
    }
    else
    {
      measured_force = -1*wrench.body2Force;
      measured_torque = -1*wrench.body2Torque;
    }
  }
  else
  {
    GZ_ASSERT(measure_frame == JOINT,
              "measure_frame must be PARENT_LINK, CHILD_LINK or JOINT");
    if (!parent_to_child)
    {
      measured_force = rotation_joint_child*wrench.body2Force;
      measured_torque = rotation_joint_child*wrench.body2Torque;
    }
    else
    {
      measured_force = rotation_joint_child*(-1*wrench.body2Force);
      measured_torque = rotation_joint_child*(-1*wrench.body2Torque);
    }
  }

  msgs::Set(this->wrenchMsg.mutable_wrench()->mutable_force(),
      measured_force);
  msgs::Set(this->wrenchMsg.mutable_wrench()->mutable_torque(),
      measured_torque);

  this->update(this->wrenchMsg);

  if (this->wrenchPub)
    this->wrenchPub->Publish(this->wrenchMsg);

  return true;
}

//////////////////////////////////////////////////
bool ForceTorqueSensor::IsActive()
{
  return Sensor::IsActive() || this->wrenchPub->HasConnections();
}
