/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/util/IntrospectionClient.hh>

bool terminate = false;

//////////////////////////////////////////////////
/// \brief Function callback executed when a SIGINT or SIGTERM signals are
/// captured. This is used to break the infinite loop that publishes messages
/// and exit the program smoothly.
void signal_handler(int _signal)
{
  if (_signal == SIGINT || _signal == SIGTERM)
    terminate = true;
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Install a signal handler for SIGINT.
  //std::signal(SIGINT, signal_handler);

  gazebo::util::IntrospectionClient client;

  // Initialize gazebo.
  //gazebo::setupServer(_argc, _argv);

  // Load a world
  //gazebo::physics::WorldPtr world = gazebo::loadWorld("worlds/empty.world");

  std::this_thread::sleep_for(std::chrono::milliseconds(3500));

  std::set<std::string> managerIds;
  managerIds = client.Managers();
  if (managerIds.empty())
  {
    std::cerr << "No managers detected" << std::endl;
    return -1;
  }

  std::string id = *managerIds.begin();

  std::set<std::string> items;
  if (!client.Items(id, items))
  {
    std::cerr << "Error in Items()" << std::endl;
    return -1;
  }

  // Create a filter for watching the pose of a box_0 model.
  std::string filterId, topic;
  if (!client.NewFilter(id, {"unit_box_0/pose", "sim_time"}, filterId, topic))
  {
    std::cerr << "Error creating filter" << std::endl;
    return -1;
  }

  // This is your custom main loop. In this example the main loop is just a
  // for loop with 2 iterations.
  //for (unsigned int i = 0; i < 2; ++i)
  //{
    // Run simulation for 100 steps.
    //gazebo::runWorld(world, 100);
  //}

  getchar();
  //while (!terminate)
  //  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  //// Make sure to shut everything down.
  //gazebo::shutdown();

  return 0;

  // Load gazebo
  //gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  //gazebo::transport::NodePtr node(new gazebo::transport::Node());
  //node->Init();

  // Publish to a Gazebo topic
  //gazebo::transport::PublisherPtr pub =
  //  node->Advertise<gazebo::msgs::Pose>("~/pose_example");

  // Wait for a subscriber to connect
  //pub->WaitForConnection();

  // Publisher loop...replace with your own code.
  //while (true)
  //{
  //  // Throttle Publication
  //  gazebo::common::Time::MSleep(100);
//
  //  // Generate a pose
  //  ignition::math::Pose3d pose(1, 2, 3, 4, 5, 6);
//
  //  // Convert to a pose message
  //  gazebo::msgs::Pose msg;
  //  gazebo::msgs::Set(&msg, pose);
//
  //  pub->Publish(msg);
  //}
//

}
