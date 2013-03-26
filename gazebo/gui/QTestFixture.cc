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

// The following is needed to enable the GetMemInfo function for OSX
#ifdef __MACH__
# include <mach/mach.h>
#endif  // __MACH__

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

void QTestFixture::LoadServer(const std::string &_worldFilename, bool _paused)
{
  // Create, load, and run the server in its own thread
  this->serverThread = new boost::thread(
      boost::bind(&QTestFixture::RunServer2, this, _worldFilename, _paused));

  // Wait for the server to come up
  // Use a 30 second timeout.
  int waitCount = 0, maxWaitCount = 3000;
  while ((!this->server || !this->server->GetInitialized()) &&
      ++waitCount < maxWaitCount)
    gazebo::common::Time::MSleep(10);

  printf("Here***\n");
  gazebo::rendering::create_scene(
      gazebo::physics::get_world()->GetName(), false);
}

/////////////////////////////////////////////////
void QTestFixture::RunServer2(const std::string &_worldFilename, bool _paused)
{
  this->server = new gazebo::Server();
  this->server->LoadFile(_worldFilename);
  this->server->Init();

  this->SetPause(_paused);

  gazebo::rendering::create_scene(
      gazebo::physics::get_world()->GetName(), false);

  this->server->Run();

  this->server->Fini();
  printf("Stop4\n");

  delete this->server;
  this->server = NULL;
  printf("Stop5\n");
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

  printf("HERE\n");
  // Create, load, and run the server in its own thread
  this->serverThread = new boost::thread(
      boost::bind(&QTestFixture::RunServer, this));

  printf("Yup\n");
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
  printf("RUN\n");
  this->server->Run();
  printf("RUN STOP\n");
}

/////////////////////////////////////////////////
void QTestFixture::SetPause(bool _pause)
{
  gazebo::physics::pause_worlds(_pause);
}

/////////////////////////////////////////////////
void QTestFixture::cleanup()
{
  printf("QTestFixture::cleanup\n");
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
    gazebo::common::Time::MSleep(100000);
    printf("Stop\n");
    this->server->Stop();
    gazebo::common::Time::MSleep(100000);

    if (this->serverThread)
    {
      printf("Stop2\n");
      this->serverThread->join();
    }
    printf("Stop2a\n");

    // gazebo::rendering::remove_scene(gazebo::physics::get_world()->GetName());
    printf("Stop3\n");

/*    this->server->Fini();
    printf("Stop4\n");

    delete this->server;
    this->server = NULL;
    printf("Stop5\n");
    */
  }

    printf("Stop6\n");
  delete this->serverThread;
  this->serverThread = NULL;
}

/////////////////////////////////////////////////
void QTestFixture::cleanupTestCase()
{
  printf("QTestFixture::cleanupTestCase\n");
  if (this->server)
  {
    this->server->Stop();

    if (this->serverThread)
    {
      this->serverThread->join();
    }

    // gazebo::rendering::remove_scene(gazebo::physics::get_world()->GetName());
    // this->server->Fini();
    // delete this->server;
    // this->server = NULL;
  }

  delete this->serverThread;
  this->serverThread = NULL;
}

/////////////////////////////////////////////////
void QTestFixture::GetMemInfo(double &_resident, double &_share)
{
  int totalSize, residentPages, sharePages;
  totalSize = residentPages = sharePages = 0;

#ifdef __linux__
  std::ifstream buffer("/proc/self/statm");
  buffer >> totalSize >> residentPages >> sharePages;
  buffer.close();

  // in case x86-64 is configured to use 2MB pages
  int64_t pageSizeKb = sysconf(_SC_PAGE_SIZE) / 1024;

  _resident = residentPages * pageSizeKb;
  _share = sharePages * pageSizeKb;
#elif __MACH__
  // /proc is only available on Linux
  // for OSX, use task_info to get resident and virtual memory
  struct task_basic_info t_info;
  mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;
  if (KERN_SUCCESS != task_info(mach_task_self(),
                                TASK_BASIC_INFO,
                                (task_info_t)&t_info,
                                &t_info_count))
  {
    gzerr << "failure calling task_info\n";
    return;
  }
  _resident = static_cast<double>(t_info.resident_size/1024);
  _share = static_cast<double>(t_info.virtual_size/1024);
#else
  gzerr << "Unsupported architecture\n";
  return;
#endif
}
