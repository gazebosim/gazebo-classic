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

#include "ActuatorPlugin.hh"

using namespace gazebo;

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

float VelocityLimiterModel(const float _speed, const float _torque,
  const ActuatorProperties &_properties)
{
  if (_speed > _properties.maximumVelocity)
    return _properties.maximumTorque;

  return _torque;
}

float NullModel(const float /*_speed*/, const float /*_torque*/,
                const ActuatorProperties &/*_properties*/)
{
  return 0;
}

void ActuatorPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Read the SDF
  if (_sdf->HasElement("actuator"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("actuator");
    while (elem)
    {
      // Get actuator properties
      ActuatorProperties properties;

      // actuator name is currently an optional property
      if (elem->HasElement("name"))
        properties.name = elem->GetElement("name")->Get<std::string>();

      if (!elem->HasElement("joint"))
        continue;
      std::string jointName = elem->GetElement("joint")->Get<std::string>();

      properties.modelFunction = NULL;
      if (elem->HasElement("type"))
      {
        std::string modelType = elem->GetElement("type")->Get<std::string>();
        if (modelType.compare("electric_motor") == 0)
        {
          if (!elem->HasElement("power") ||
              !elem->HasElement("max_velocity") ||
              !elem->HasElement("max_torque"))
            continue;

          properties.power = elem->GetElement("power")->Get<float>();
          properties.maximumVelocity =
            elem->GetElement("max_velocity")->Get<float>();
          properties.maximumTorque =
            elem->GetElement("max_torque")->Get<float>();

          properties.modelFunction = ElectricMotorModel;
        }
        else if (modelType.compare("velocity_limiter") == 0)
        {
          if (!elem->HasElement("max_velocity") ||
              !elem->HasElement("max_torque"))
            continue;
          properties.maximumVelocity =
            elem->GetElement("max_velocity")->Get<float>();
          properties.maximumTorque =
            elem->GetElement("max_torque")->Get<float>();
          properties.modelFunction = VelocityLimiterModel;
        }
        else
        {
          properties.modelFunction = NullModel;
        }
      }
      if (properties.modelFunction == NULL)
        properties.modelFunction = NullModel;

      if (elem->HasElement("index"))
      {
        properties.jointIndex = elem->GetElement("index")->Get<unsigned int>();
      }
      else
      {
        properties.jointIndex = 0;
      }

      // Store pointer to the joint we will actuate
      physics::JointPtr joint = _parent->GetJoint(jointName);
      if (!joint)
        continue;
      joint->SetEffortLimit(properties.jointIndex, properties.maximumTorque);
      this->joints.push_back(joint);
      this->actuators.push_back(properties);

      elem = elem->GetNextElement("actuator");
    }
    // Set up a physics update callback
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ActuatorPlugin::WorldUpdateCallback, this)));
  }
}

void ActuatorPlugin::WorldUpdateCallback()
{
  // Update the stored joints according to the desired model.
  for (unsigned int i = 0; i < this->joints.size(); i++)
  {
    const int index = this->actuators[i].jointIndex;
    const float velocity = this->joints[i]->GetVelocity(index);
    float curForce = this->joints[i]->GetForce(index);

    /*
    if (curForce > maxForce)
    {
      this->joints[i]->SetForce(index, maxForce);
    }*/
    float maxForce = this->actuators[i].modelFunction(velocity, curForce,
              this->actuators[i]);
    this->joints[i]->SetEffortLimit(index, maxForce);
  }
}
