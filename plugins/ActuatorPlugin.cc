/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "ActuatorPlugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
/// \brief Calculate torque due to the input conditions for an electric motor
/// model. Simplified from http://lancet.mit.edu/motors/motors3.html#power
/// \param[in] _speed Input velocity.
/// \param[in] _properties Static properties of this actuator
/// \return Torque according to the model.
float ElectricMotorModel(const float _speed, const float /*_torque*/,
  const ActuatorProperties &_properties)
{
  if (_speed > _properties.maximumVelocity)
    return _properties.power / _properties.maximumVelocity;

  float torque = _properties.power / _speed;

  if (torque > _properties.maximumTorque)
    return _properties.maximumTorque;

  return torque;
}

//////////////////////////////////////////////////
/// \brief A simple velocity limiting motor model. Returns the maximum torque
/// if speed is above the allowed limit, and returns the input torque otherwise.
/// \param[in] _speed Input velocity.
/// \param[in] _torque Input torque.
/// \param[in] _properties Static properties of this actuator
/// \return Torque according to the model.
float VelocityLimiterModel(const float _speed, const float _torque,
  const ActuatorProperties &_properties)
{
  if (_speed > _properties.maximumVelocity)
    return _properties.maximumTorque;

  return _torque;
}

//////////////////////////////////////////////////
/// \brief The null motor model. Nothing exciting happening here.
/// \return Torque according to the model, which will always be zero.
float NullModel(const float /*_speed*/, const float /*_torque*/,
                const ActuatorProperties &/*_properties*/)
{
  return 0;
}

//////////////////////////////////////////////////
void ActuatorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Read the SDF
  if (_sdf->HasElement("actuator"))
  {
    for (sdf::ElementPtr elem = _sdf->GetElement("actuator"); elem != NULL;
         elem = elem->GetNextElement("actuator"))
    {
      // Get actuator properties
      ActuatorProperties properties;

      // actuator name is currently an optional property
      if (elem->HasElement("name"))
        properties.name = elem->Get<std::string>("name");

      if (!elem->HasElement("joint"))
      {
        gzwarn << "Invalid SDF: got actuator element without joint."
               << std::endl;
        continue;
      }
      std::string jointName = elem->Get<std::string>("joint");

      properties.modelFunction = NullModel;
      if (elem->HasElement("type"))
      {
        std::string modelType = elem->Get<std::string>("type");
        if (modelType.compare("electric_motor") == 0)
        {
          if (!elem->HasElement("power") ||
              !elem->HasElement("max_velocity") ||
              !elem->HasElement("max_torque"))
          {
            gzwarn << "Invalid SDF: Missing required elements for motor model "
                   << modelType << "." << std::endl;
            continue;
          }

          properties.power = elem->Get<float>("power");
          properties.maximumVelocity = elem->Get<float>("max_velocity");
          properties.maximumTorque = elem->Get<float>("max_torque");

          properties.modelFunction = ElectricMotorModel;
        }
        else if (modelType.compare("velocity_limiter") == 0)
        {
          if (!elem->HasElement("max_velocity") ||
              !elem->HasElement("max_torque"))
          {
            gzwarn << "Invalid SDF: Missing required elements for motor model "
                   << modelType << "." << std::endl;
            continue;
          }
          properties.maximumVelocity = elem->Get<float>("max_velocity");
          properties.maximumTorque = elem->Get<float>("max_torque");
          properties.modelFunction = VelocityLimiterModel;
        }
        else if (modelType.compare("null") != 0)
        {
          gzwarn << "Unknown motor model specified, selecting NullModel."
                 << std::endl;
        }
      }
      else
      {
        gzwarn << "No motor model specified, selecting NullModel."
               << std::endl;
      }

      if (elem->HasElement("index"))
      {
        properties.jointIndex = elem->Get<unsigned int>("index");
      }
      else
      {
        properties.jointIndex = 0;
      }

      // Store pointer to the joint we will actuate
      physics::JointPtr joint = _parent->GetJoint(jointName);
      if (!joint)
      {
        gzwarn << "Invalid SDF: actuator joint " << jointName << " does not "
               << "exist!" << std::endl;
        continue;
      }
      joint->SetEffortLimit(properties.jointIndex, properties.maximumTorque);
      this->joints.push_back(joint);
      this->actuators.push_back(properties);
    }
    // Set up a physics update callback
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ActuatorPlugin::WorldUpdateCallback, this)));
  }
}

//////////////////////////////////////////////////
void ActuatorPlugin::WorldUpdateCallback()
{
  // Update the stored joints according to the desired model.
  for (unsigned int i = 0; i < this->joints.size(); i++)
  {
    const int index = this->actuators[i].jointIndex;
    const float velocity = this->joints[i]->GetVelocity(index);
    float curForce = this->joints[i]->GetForce(index);
    float maxForce = this->actuators[i].modelFunction(velocity, curForce,
              this->actuators[i]);
    this->joints[i]->SetEffortLimit(index, maxForce);
  }
}
