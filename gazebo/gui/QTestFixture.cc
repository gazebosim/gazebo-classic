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

#include "gazebo/physics/Physics.hh"

#include "gazebo/rendering/Rendering.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/gui/Gui.hh"
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

  path = TEST_PATH;
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path);
}

/////////////////////////////////////////////////
void QTestFixture::init()
{
  this->resMaxPercentChange = 2.5;
  this->shareMaxPercentChange = 1.0;

  this->serverThread = NULL;
  this->GetMemInfo(this->residentStart, this->shareStart);
}

/////////////////////////////////////////////////
void QTestFixture::Load(const std::string &_worldFilename, bool _paused)
{
  this->server = new gazebo::Server();
  this->server->LoadFile(_worldFilename);
  this->server->Init();

  this->SetPause(_paused);

  gazebo::rendering::create_scene(
      gazebo::physics::get_world()->GetName(), false);

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
  this->server->Run();
}

/////////////////////////////////////////////////
void QTestFixture::SetPause(bool _pause)
{
  gazebo::physics::pause_worlds(_pause);
}

/////////////////////////////////////////////////
void QTestFixture::cleanup()
{
  double residentEnd, shareEnd;
  this->GetMemInfo(residentEnd, shareEnd);

  // Calculate the percent change from the initial resident and shared
  // memory
  double resPercentChange = (residentEnd - residentStart) / residentStart;
  double sharePercentChange = (shareEnd - shareStart) / shareStart;

  std::cout << "ResPercentChange[" << resPercentChange << "]\n";
  // Make sure the percent change values are reasonable.
  QVERIFY(resPercentChange < this->resMaxPercentChange);
  QVERIFY(sharePercentChange < this->shareMaxPercentChange);

  if (this->server)
  {
    this->server->Stop();

    if (this->serverThread)
    {
      this->serverThread->join();
    }

    gazebo::rendering::remove_scene(gazebo::physics::get_world()->GetName());

    this->server->Fini();

    delete this->server;
    this->server = NULL;
  }

  delete this->serverThread;
  this->serverThread = NULL;
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

    gazebo::rendering::remove_scene(gazebo::physics::get_world()->GetName());

    this->server->Fini();

    delete this->server;
    this->server = NULL;
  }

  delete this->serverThread;
  this->serverThread = NULL;
}

/////////////////////////////////////////////////
void QTestFixture::GetMemInfo(double &_resident, double &_share)
{
  int totalSize, residentPages, sharePages;
  totalSize = residentPages = sharePages = 0;

  std::ifstream buffer("/proc/self/statm");
  buffer >> totalSize >> residentPages >> sharePages;
  buffer.close();

  // in case x86-64 is configured to use 2MB pages
  int64_t pageSizeKb = sysconf(_SC_PAGE_SIZE) / 1024;

  _resident = residentPages * pageSizeKb;
  _share = sharePages * pageSizeKb;
}
