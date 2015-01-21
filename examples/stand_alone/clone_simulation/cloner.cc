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

#include <cstdlib>
#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

/////////////////////////////////////////////////
void OnWorldModify(ConstWorldModifyPtr &_msg)
{
  if (_msg->has_cloned() && _msg->cloned() && _msg->has_cloned_uri())
  {
    std::cout << "World cloned. You can connect a client by typing\n"
              << "\tGAZEBO_MASTER_URI=" << _msg->cloned_uri()
              << " gzclient" << std::endl;
  }
}

/////////////////////////////////////////////////
void RunServer()
{
  // Initialize gazebo server.
  boost::scoped_ptr<gazebo::Server> server(new gazebo::Server());
  try
  {
    if (!server->ParseArgs(0, NULL))
      return;

    // Initialize the informational logger. This will log warnings, and errors.
    gzLogInit("server-", "gzserver.log");

    server->Run();
    server->Fini();
  }
  catch(gazebo::common::Exception &_e)
  {
    _e.Print();
    server->Fini();
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Launch a server in a different thread.
  boost::thread serverThread(RunServer);

  // Create a node for communication.
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publisher to the server control.
  gazebo::transport::PublisherPtr serverControlPub =
    node->Advertise<gazebo::msgs::ServerControl>("/gazebo/server/control");

  // Subscriber to receive world updates (e.g.: a notification after a cloning).
  gazebo::transport::SubscriberPtr worldModSub =
    node->Subscribe("/gazebo/world/modify", &OnWorldModify);

  std::cout << "\nPress [ENTER] to clone the current simulation\n" << std::endl;
  getchar();

  // Clone the server programmatically.
  gazebo::msgs::ServerControl msg;
  msg.set_save_world_name("");
  msg.set_clone(true);
  msg.set_new_port(11346);
  serverControlPub->Publish(msg);

  // Wait for the simulation clone before showing the next message.
  gazebo::common::Time::MSleep(200);

  std::cout << "\nPress [ENTER] to exit and kill all the servers." << std::endl;
  getchar();

  // Make sure to shut everything down.
  std::string cmd = "kill -15 `ps -A | grep -m1 gzserver | awk '{print $1}'`";
  int ret = std::system(cmd.c_str());
  if (ret != 0)
    std::cerr << "kill gzserver returned a non zero value:" << ret << std::endl;

  gazebo::shutdown();
}
