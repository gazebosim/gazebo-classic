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

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Example-specific parameters:
  const int maxIterations = 600;
  const int sampleTimesteps = 1;
  const float maxTorqueAdj = 150;

  unsigned int index;
  float maxTorque;

  // Initialize gazebo.
  gazebo::setupServer(_argc, _argv);

  // Load a world with two models: one actuated, one not
  gazebo::physics::WorldPtr world =
    gazebo::loadWorld("../actuator_example.world");
  if (!world)
  {
    std::cout << "Could not load world actuator_example" << std::endl;
    return -1;
  }

  // Get the models and pointers to their joints
  std::vector<std::string> modelNames;
  modelNames.push_back("actuator_example");
  modelNames.push_back("unactuated_example");

  std::string jointName;

  std::vector<gazebo::physics::JointPtr> joints;

  for (unsigned int i = 0; i < modelNames.size(); i++)
  {
    gazebo::physics::ModelPtr model = world->GetModel(modelNames[i]);
    if (!model)
    {
      std::cout << "Couldn't find model: " << modelNames[i] << std::endl;
      return -1;
    }

    if (modelNames[i].compare("actuator_example") == 0)
    {
      const sdf::ElementPtr modelSDF = model->GetSDF();
      // Find the ActuatorPlugin SDF block
      if (!modelSDF->HasElement("plugin"))
      {
        std::cout << "ERROR: couldn't find index element." << std::endl;
        return -1;
      }
      sdf::ElementPtr elem = modelSDF->GetElement("plugin");
      while (elem->GetAttribute("filename")->GetAsString().compare("libActuatorPlugin.so"))
      {
        elem = elem->GetNextElement("plugin");
      }

      if (!elem->HasElement("actuator"))
      {
        std::cout << "ERROR: couldn't find actuator element" << std::endl;
        return -1;
      }
      elem = elem->GetElement("actuator");
      if (!elem->HasElement("index"))
      {
        std::cout << "ERROR: couldn't find index element." << std::endl;
        return -1;
      }
      index = elem->GetElement("index")->Get<unsigned int>();
      if (!elem->HasElement("max_torque"))
      {
        std::cout << "ERROR: couldn't find max_torque element." << std::endl;
        return -1;
      }
      maxTorque = maxTorqueAdj*elem->GetElement("max_torque")->Get<float>();

      if (!elem->HasElement("joint"))
      {
        std::cout << "ERROR: couldn't find joint element." << std::endl;
        return -1;
      }
      jointName = elem->GetElement("joint")->Get<std::string>();
    }
    
    gazebo::physics::JointPtr joint = model->GetJoint(jointName);
    if (!joint)
    {
      std::cout << "Couldn't find joint " << jointName << " for model "
                << modelNames[i] << std::endl;
    }
    else
    {
      joints.push_back(joint);
    }
  }

  // Open a file for writing
  boost::filesystem::path path("../data");
  if (!boost::filesystem::exists(path))
  {
    boost::filesystem::create_directories(path);
  }
  std::ofstream fileStream;
  fileStream.open((path.string() + "/data.csv").c_str());
  // Push initial file headings
  fileStream << "actuated_joint_pos\tactuated_joint_vel\t"
             << "actuated_joint_torque\tunactuated_joint_pos\t"
             << "unactuated_joint_vel\tunactuated_joint_torque" << std::endl;

  // Run the simulation for a fixed number of iterations.

  for (unsigned int i = 0; i < maxIterations; ++i)
  {
    float currentTorque = maxTorque*i/maxIterations;
    for (unsigned int j = 0; j < joints.size(); ++j)
    {
      if (!joints[j])
      {
        std::cout << "got NULL joint in actuator example main.cc" << std::endl;
        continue;
      }

      joints[j]->SetForce(index, currentTorque);
    }

    gazebo::runWorld(world, sampleTimesteps);

    // Print joint position, velocity and torques for each model to file
    for (unsigned int j = 0; j < joints.size(); ++j)
    {
      fileStream << joints[j]->GetAngle(index) << "\t"
                 << joints[j]->GetVelocity(index) << "\t"
                 << joints[j]->GetForce(index);
      if (j == joints.size() - 1)
      {
        fileStream << std::endl;
      }
      else
      {
        fileStream << "\t";
      }
    }
  }

  // Close everything.
  fileStream.close();
  gazebo::shutdown();
  return 0;
}
