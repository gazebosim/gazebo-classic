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

#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/util/LogRecord.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/gui/QTestFixture.hh"

/////////////////////////////////////////////////
QTestFixture::QTestFixture()
  : server(NULL), serverThread(NULL), residentStart(0), shareStart(0)
{
}

/////////////////////////////////////////////////
void QTestFixture::initTestCase()
{
  // Initialize the informational logger. This will log warnings, and
  // errors.
  gazebo::common::Console::Instance()->Init("test.log");

  // Initialize the data logger. This will log state information.
  gazebo::util::LogRecord::Instance()->Init("test");

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
  this->serverThread = NULL;
  this->GetMemInfo(this->residentStart, this->shareStart);
}

/////////////////////////////////////////////////
void QTestFixture::Load(const std::string &_worldFilename, bool _paused)
{
  // Create, load, and run the server in its own thread
  this->serverThread = new boost::thread(
      boost::bind(&QTestFixture::RunServer, this, _worldFilename, _paused));

  // Wait for the server to come up
  // Use a 30 second timeout.
  int waitCount = 0, maxWaitCount = 3000;
  while ((!this->server || !this->server->GetInitialized()) &&
      ++waitCount < maxWaitCount)
    gazebo::common::Time::MSleep(10);
}

/////////////////////////////////////////////////
void QTestFixture::RunServer(const std::string &_worldFilename, bool _paused)
{
  this->server = new gazebo::Server();
  this->server->LoadFile(_worldFilename);
  this->server->Init();

  gazebo::rendering::create_scene(
      gazebo::physics::get_world()->GetName(), false);

  this->SetPause(_paused);

  this->server->Run();
  printf("Server not running\n");

  gazebo::rendering::remove_scene(gazebo::physics::get_world()->GetName());

  this->server->Fini();
  delete this->server;
  this->server = NULL;
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

  std::cout << "REs[" << resPercentChange << "]\n";
  std::cout << "Shared[" << sharePercentChange << "]\n";
  // Make sure the percent change values are reasonable.
  QVERIFY(resPercentChange < 2.5);
  QVERIFY(sharePercentChange < 1.0);

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

/////////////////////////////////////////////////
void QTestFixture::cleanupTestCase()
{
}

/////////////////////////////////////////////////
void QTestFixture::GetMemInfo(double &_resident, double &_share)
{
#ifdef __linux__
  int totalSize, residentPages, sharePages;
  totalSize = residentPages = sharePages = 0;

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
