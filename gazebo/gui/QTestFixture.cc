/*
 * Copyright 2011 Nate Koenig
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

#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/gui/QTestFixture.hh"

/////////////////////////////////////////////////
void QTestFixture::initTestCase()
{
  // Initialize the informational logger. This will log warnings, and
  // errors.
  gazebo::common::Console::Instance()->Init("test.log");

  // Initialize the data logger. This will log state information.
  gazebo::common::LogRecord::Instance()->Init("test");

  // Add local search paths
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(PROJECT_SOURCE_PATH);

  std::string path = PROJECT_SOURCE_PATH;
  path += "/sdf/worlds";
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path);

  // Create, load, and run the server in its own thread
  this->serverThread = new boost::thread(
      boost::bind(&QTestFixture::RunServer, this));

  // Wait for the server to come up
  // Use a 30 second timeout.
  int waitCount = 0, maxWaitCount = 3000;
  while ((!this->server || !this->server->GetInitialized()) &&
      ++waitCount < maxWaitCount)
    gazebo::common::Time::MSleep(10);
}

/////////////////////////////////////////////////
void QTestFixture::RunServer()
{
  this->server = new gazebo::Server();
  this->server->LoadFile("empty.world");
  this->server->Init();

  gazebo::rendering::create_scene(
      gazebo::physics::get_world()->GetName(), false);

  // this->SetPause(false);

  this->server->Run();

  gazebo::rendering::remove_scene(gazebo::physics::get_world()->GetName());

  this->server->Fini();
  delete this->server;
  this->server = NULL;
}

/////////////////////////////////////////////////
void QTestFixture::cleanupTestCase()
{
  if (this->server)
  {
    this->server->Stop();

    if (this->serverThread)
    {
      this->serverThread->join();
    }
  }

  delete this->serverThread;
  this->serverThread = NULL;
}
