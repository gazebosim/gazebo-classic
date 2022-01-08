/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <boost/bind/bind.hpp>
#include <unistd.h>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/QTestFixture.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/util/LogRecord.hh"

/////////////////////////////////////////////////
#ifdef _WIN32
static int setenv(const char *envname, const char *envval, int overwrite)
{
  char *original = getenv(envname);
  if (!original || !!overwrite)
  {
    std::string envstring = std::string(envname) + "=" + envval;
    return _putenv(envstring.c_str());
  }
  return 0;
}
#endif

/////////////////////////////////////////////////
QTestFixture::QTestFixture()
  : server(NULL), serverThread(NULL),
    resMaxPercentChange(0), shareMaxPercentChange(0),
    residentStart(0), shareStart(0)
{
}

/////////////////////////////////////////////////
void QTestFixture::initTestCase()
{
  // Verbose mode
  gazebo::common::Console::SetQuiet(false);

  // Initialize the informational logger. This will log warnings, and
  // errors.
  gzLogInit("qtest-", "test.log");

  // Initialize the data logger. This will log state information.
  gazebo::util::LogRecord::Instance()->Init("test");

  // Add local search paths
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(PROJECT_SOURCE_PATH);

  std::string path = PROJECT_SOURCE_PATH;
  path += "/worlds";
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path);

  path = TEST_PATH;
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path);
}

/////////////////////////////////////////////////
void QTestFixture::init()
{
  // Set environment variable
  char *env = getenv("IN_TESTSUITE");
  QVERIFY(env == nullptr);

  setenv("IN_TESTSUITE", "1", 1);

  env = getenv("IN_TESTSUITE");
  QVERIFY(env != nullptr);

  this->resMaxPercentChange = 3.0;
  this->shareMaxPercentChange = 1.0;

  this->serverThread = NULL;
  this->GetMemInfo(this->residentStart, this->shareStart);
  gazebo::rendering::load();

  if (!gazebo::gui::register_metatypes())
    gzerr << "Unable to register Qt metatypes" << std::endl;
}

/////////////////////////////////////////////////
void QTestFixture::Load(const std::string &_worldFilename, bool _paused,
    bool _serverScene, bool _clientScene)
{
#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0)) && defined(__APPLE__)
  this->resMaxPercentChange *= 2.0;
#endif

  // Create, load, and run the server in its own thread
  this->serverThread = new boost::thread(
      boost::bind(&QTestFixture::RunServer, this,
        _worldFilename, _paused, _serverScene));

  // Wait for the server to come up
  // Use a 60 second timeout.
  int waitCount = 0, maxWaitCount = 6000;
  while ((!this->server || !this->server->GetInitialized()) &&
      ++waitCount < maxWaitCount)
    gazebo::common::Time::MSleep(10);

  if (!this->server || !this->server->GetInitialized() ||
      waitCount >= maxWaitCount)
  {
    gzerr << "Unable to initialize server. Potential reasons:" << std::endl;
    gzerr << "\tIncorrect world name?" << std::endl;
    gzerr << "\tConnection problem downloading models" << std::endl;
    return;
  }

  if (_clientScene)
  {
    gazebo::rendering::create_scene(
        gazebo::physics::get_world()->Name(), true);
  }
}

/////////////////////////////////////////////////
void QTestFixture::RunServer(const std::string &_worldFilename,
    bool _paused, bool _createScene)
{
  this->server = new gazebo::Server();
  this->server->PreLoad();
  this->server->LoadFile(_worldFilename);

  this->SetPause(_paused);

  if (_createScene)
  {
    gazebo::rendering::create_scene(
        gazebo::physics::get_world()->Name(), false);
  }

  this->server->Run();

  delete this->server;
  this->server = NULL;
}

/////////////////////////////////////////////////
void QTestFixture::SetPause(bool _pause)
{
  gazebo::physics::pause_worlds(_pause);
}

/////////////////////////////////////////////////
void QTestFixture::ProcessEventsAndDraw(QMainWindow *_mainWindow,
    const unsigned int _repeat, const unsigned int _ms)
{
  for (size_t i = 0; i < _repeat; ++i)
  {
    gazebo::common::Time::MSleep(_ms);
    QCoreApplication::processEvents();
    if (_mainWindow)
      _mainWindow->repaint();
  }
}

/////////////////////////////////////////////////
void QTestFixture::cleanup()
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

  gazebo::rendering::fini();

  double residentEnd, shareEnd;
  this->GetMemInfo(residentEnd, shareEnd);

  // Calculate the percent change from the initial resident and shared memory
  double resPercentChange = (residentEnd - residentStart) / residentStart;
  double sharePercentChange = (shareEnd - shareStart) / shareStart;

  std::cout << "SharePercentChange[" << sharePercentChange << "] "
    << "ShareMaxPercentChange[" << this->shareMaxPercentChange << "]\n";
  std::cout << "ResPercentChange[" << resPercentChange << "]"
    << "ResMaxPercentChange[" << this->resMaxPercentChange << "]\n";

  // Make sure the percent change values are reasonable.
  QVERIFY(resPercentChange < this->resMaxPercentChange);
  QVERIFY(sharePercentChange < this->shareMaxPercentChange);

  // Unset environment variable
  char *env = getenv("IN_TESTSUITE");
  QVERIFY(env != nullptr);

  unsetenv("IN_TESTSUITE");

  env = getenv("IN_TESTSUITE");
  QVERIFY(env == nullptr);
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
