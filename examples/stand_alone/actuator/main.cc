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
  const int maxIterations = 1000;
  const int extraIterations = 5;
  const int sampleTimesteps = 10;

  // TODO: read index and maximumTargetVelocity from SDF
  const int index = 0;
  const float maximumTargetVelocity = 500;
  const float velocityStep = maximumTargetVelocity / (float) maxIterations;

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

  const std::string jointName = "JOINT_0";

  std::vector<gazebo::physics::JointPtr> joints;

  for (unsigned int i = 0; i < modelNames.size(); i++)
  {
    gazebo::physics::ModelPtr model = world->GetModel(modelNames[i]);
    if (!model)
    {
      std::cout << "Couldn't find model: " << modelNames[i] << std::endl;
      continue;
    }
    gazebo::physics::JointPtr joint = model->GetJoint(jointName);
    if (!joint)
    {
      std::cout << "Couldn't find joint: " << jointName << std::endl;
      continue;
    }
    joints.push_back(joint);
  }

  // Open a file for writing
  boost::filesystem::path path("../data/data.csv");
  if (!boost::filesystem::exists(path))
  {
    boost::filesystem::create_directories(path);
  }
  std::ofstream fileStream;
  fileStream.open(path.string().c_str());
  // Push initial file headings
  fileStream << "actuated_joint_pos\tactuated_joint_vel\t"
             << "actuated_joint_torque\tunactuated_joint_pos\t"
             << "unactuated_joint_vel\tunactuated_joint_torque" << std::endl;

  // Run the simulation for a fixed number of iterations.
  // Apply an increasing ramp signal to the velocity of the joints of interest
  for (unsigned int i = 0; i < maxIterations+extraIterations; ++i)
  {
    // Command joint speed for this timestep
    for (unsigned int j = 0; j < joints.size(); j++)
    {
      if (!joints[i])
      {
        std::cout << "got NULL joint in actuator example main.cc" << std::endl;
        continue;
      }
      joints[i]->SetVelocity(index, velocityStep*i);
    }

    gazebo::runWorld(world, sampleTimesteps);

    // Print joint position, velocity and torques for each model to file
    for (unsigned int j = 0; j < joints.size(); j++)
    {
      fileStream << joints[i]->GetAngle(index) << "\t"
                 << joints[i]->GetVelocity(index) << "\t"
                 << joints[i]->GetForce(index);
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
