/*
 * Copyright 2011 Nate Koenig
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

#include "common/Events.hh"
#include "common/Time.hh"
#include "common/Console.hh"
#include "common/Logger.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
Logger::Logger()
{
  this->stop = false;

  // Get the user's home directory
  char *homePath = getenv("HOME");
  if (!homePath)
    this->logPathname = "/tmp/gazebo";
  else
    this->logPathname = homePath;

  this->logPathname += "/.gazebo/log/";

  // Add the current time
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
  this->logPathname += boost::posix_time::to_iso_extended_string(now);
}

//////////////////////////////////////////////////
bool Logger::Init(const std::string &_subdir)
{
  if (!_subdir.empty())
    this->logPathname += "/" + _subdir;

  // Create the log directory
  boost::filesystem::path path(this->logPathname);
  if (!boost::filesystem::exists(path))
    boost::filesystem::create_directories(path);

  // Set the data log filename
  this->dataLogFilename = this->logPathname + "/data.log";

  // Set the log filename for output from gz* macros.
  this->gzoutLogFilename = this->logPathname + "/gzout.log";

  // Listen to the world update event
  if (!this->updateConnection)
  {
    this->updateConnection =
      event::Events::ConnectWorldUpdateStart(
          boost::bind(&Logger::Update, this));
  }
  else
  {
    gzerr << "Logger has already been initialized\n";
    return false;
  }

  // Start the logging thread
  if (!this->writeThread)
    this->writeThread = new boost::thread(boost::bind(&Logger::Run, this));
  else
  {
    gzerr << "Logger has already been initialized\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
Logger::~Logger()
{
  // Stop the write thread.
  this->Stop();

  // Delete all the log objects
  for (std::vector<LogObj*>::iterator iter = this->logObjects.begin();
       iter != this->logObjects.end(); ++iter)
  {
    delete *iter;
  }
  this->logObjects.clear();
}

//////////////////////////////////////////////////
void Logger::Start()
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  // Check to see if the logger is already started.
  if (!this->stop)
    return;

  // Start the log write thread
  this->stop = false;
  this->writeThread = new boost::thread(boost::bind(&Logger::Run, this));

  // Listen to the world update signal
  this->updateConnection =
    event::Events::ConnectWorldUpdateStart(boost::bind(&Logger::Update, this));
}

//////////////////////////////////////////////////
void Logger::Stop()
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  this->stop = true;

  // Kick the write thread
  this->dataAvailableCondition.notify_one();

  // Wait for the write thread, if it exists
  if (this->writeThread)
    this->writeThread->join();
  delete this->writeThread;
  this->writeThread = NULL;

  // Disconnect from the world update signale
  if (this->updateConnection)
    event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->updateConnection.reset();
}

//////////////////////////////////////////////////
bool Logger::Add(const std::string &_object,
                 boost::function<bool (std::ostringstream &)> _logCallback)
{
  // Use the default data log filename
  return this->Add(_object, this->dataLogFilename, _logCallback);
}

//////////////////////////////////////////////////
bool Logger::Add(const std::string &_object, const std::string &_filename,
                 boost::function<bool (std::ostringstream &)> _logCallback)
{
  // Create a new log object
  Logger::LogObj *newLog = new Logger::LogObj(_object, _filename, _logCallback);

  // Make sure the log object is valid
  if (newLog->valid)
    this->logObjects.push_back(newLog);
  else
  {
    gzerr << "Unable to create log. File permissions are probably bad.\n";
    delete newLog;
  }

  // Update the pointer to the end of the log objects list.
  this->logObjectsEnd = this->logObjects.end();
  return true;
}

//////////////////////////////////////////////////
bool Logger::Remove(const std::string &_object)
{
  std::vector<LogObj*>::iterator iter;

  // Find the log object to remove
  for (iter = this->logObjects.begin(); iter != this->logObjects.end(); ++iter)
  {
    if ((*iter)->GetName() == _object)
    {
      delete *iter;
      this->logObjects.erase(iter);
      break;
    }
  }

  // Update the pointer to the end of the log objects list.
  this->logObjectsEnd = this->logObjects.end();
  return true;
}

//////////////////////////////////////////////////
void Logger::Update()
{
  {
    boost::mutex::scoped_lock lock(this->writeMutex);

    // Collect all the new log data. This will not write data to disk.
    for (this->updateIter = this->logObjects.begin();
        this->updateIter != this->logObjectsEnd; ++this->updateIter)
    {
      (*this->updateIter)->Update();
    }
  }

  // Signal that new data is available.
  this->dataAvailableCondition.notify_one();
}

//////////////////////////////////////////////////
void Logger::Run()
{
  // This loop will write data to disk.
  while (!this->stop)
  {
    {
      // Wait for new data.
      boost::mutex::scoped_lock lock(this->writeMutex);
      this->dataAvailableCondition.wait(lock);

      // Write all the data
      for (this->updateIter = this->logObjects.begin();
          this->updateIter != this->logObjectsEnd; ++this->updateIter)
      {
        (*this->updateIter)->Write();
      }
    }

    // Throttle the write loop.
    common::Time::MSleep(1000);
  }
}

//////////////////////////////////////////////////
Logger::LogObj::LogObj(const std::string &_name,
                       const std::string &_filename,
                       boost::function<bool (std::ostringstream&)> _logCB)
: valid(false)
{
  this->logFile.open(_filename.c_str(), std::fstream::out);
  this->name = _name;
  this->logCB = _logCB;

  if (!this->logFile.is_open())
  {
    gzerr << "Unable to open file for logging:" << _filename << "\n";
    return;
  }

  this->logFile << "# Global_Sim_Time Global_Real_Time Accum_Sim_Time " <<
    "Accum_Real_Time X Y Z Roll Pitch Yaw Linear_Vel_X Linear_Vel_Y " <<
    "Linear_Vel_Z Angular_Vel_Z Angular_Vel_Y Angular_Vel_Z\n";

  this->valid = true;
}

//////////////////////////////////////////////////
Logger::LogObj::~LogObj()
{
  this->logFile.close();
}

//////////////////////////////////////////////////
void Logger::LogObj::Update()
{
  std::ostringstream stream;
  if (!this->logCB(stream))
    gzerr << "Unable to update log object[" << this->name << "]\n";
  else
    this->buffer.append(stream.str());
}

//////////////////////////////////////////////////
void Logger::LogObj::Write()
{
  this->logFile.write(this->buffer.c_str(), this->buffer.size());
  this->buffer.clear();
}

//////////////////////////////////////////////////
const std::string &Logger::LogObj::GetName() const
{
  return this->name;
}
