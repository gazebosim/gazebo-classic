/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/date_time.hpp>

#include "gazebo/math/Rand.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Base64.hh"
#include "gazebo/util/LogRecord.hh"

#include "gazebo/gazebo_config.h"

using namespace gazebo;
using namespace util;

//////////////////////////////////////////////////
LogRecord::LogRecord()
{
  this->pauseState = false;
  this->running = false;
  this->paused = false;
  this->initialized = false;
  this->stopThread = false;
  this->firstUpdate = true;
  this->readyToStart = false;

  // Get the user's home directory
  // \todo getenv is not portable, and there is no generic cross-platform
  // method. Must check OS and choose a method
  char *homePath = getenv("HOME");
  GZ_ASSERT(homePath, "HOME environment variable is missing");

  if (!homePath)
    this->logBasePath = boost::filesystem::path("/tmp/gazebo");
  else
    this->logBasePath = boost::filesystem::path(homePath);

  this->logBasePath /= "/.gazebo/log/";

  this->logsEnd = this->logs.end();

  this->connections.push_back(
     event::Events::ConnectPause(
       boost::bind(&LogRecord::OnPause, this, _1)));
}

//////////////////////////////////////////////////
void LogRecord::OnPause(bool _pause)
{
  this->pauseState = _pause;
}

//////////////////////////////////////////////////
LogRecord::~LogRecord()
{
  // Stop the write thread.
  this->Fini();
}

//////////////////////////////////////////////////
bool LogRecord::Init(const std::string &_subdir)
{
  if (_subdir.empty())
  {
    gzerr << "LogRecord initialization directory is empty." << std::endl;
    return false;
  }

  this->logSubDir = _subdir;

  this->ClearLogs();

  this->initialized = true;
  this->running = false;
  this->paused = false;
  this->stopThread = false;
  this->firstUpdate = true;
  this->readyToStart = true;

  return true;
}

//////////////////////////////////////////////////
bool LogRecord::Start(const std::string &_encoding, const std::string &_path)
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  // Make sure ::Init has been called.
  if (!this->initialized)
  {
    gzerr << "LogRecord has not been initialized." << std::endl;
    return false;
  }

  // Check to see if the logger is already started.
  if (this->running)
  {
    /// \TODO replace this with gzlog
    gzerr << "LogRecord has already been started" << std::endl;
    return false;
  }

  // Get the current time as an ISO string.
  std::string logTimeDir = common::Time::GetWallTimeAsISOString();

  // Override the default path settings if the _path parameter is set.
  if (!_path.empty())
  {
    this->logBasePath = boost::filesystem::path(_path);
    this->logCompletePath = this->logBasePath;
  }
  else
    this->logCompletePath = this->logBasePath / logTimeDir / this->logSubDir;

  // Create the log directory if necessary
  if (!boost::filesystem::exists(this->logCompletePath))
    boost::filesystem::create_directories(logCompletePath);

  if (_encoding != "bz2" && _encoding != "txt" && _encoding != "zlib")
    gzthrow("Invalid log encoding[" + _encoding +
            "]. Must be one of [bz2, zlib, txt]");

  this->encoding = _encoding;

  {
    boost::mutex::scoped_lock logLock(this->writeMutex);
    this->logsEnd = this->logs.end();

    // Start all the logs
    for (Log_M::iterator iter = this->logs.begin();
         iter != this->logsEnd; ++iter)
      iter->second->Start(logCompletePath);
  }

  this->running = true;
  this->paused = false;
  this->firstUpdate = true;
  this->stopThread = false;
  this->readyToStart = false;

  this->startTime = this->currTime = common::Time();

  // Create a thread to cleanup recording.
  this->cleanupThread = boost::thread(boost::bind(&LogRecord::Cleanup, this));
  // Wait for thread to start
  this->startThreadCondition.wait(lock);

  // Start the update thread if it has not already been started
  if (!this->updateThread)
  {
    boost::mutex::scoped_lock updateLock(this->updateMutex);
    this->updateThread = new boost::thread(
        boost::bind(&LogRecord::RunUpdate, this));
    this->startThreadCondition.wait(updateLock);
  }

  // Start the writing thread if it has not already been started
  if (!this->writeThread)
  {
    boost::mutex::scoped_lock writeLock(this->runWriteMutex);
    this->writeThread = new boost::thread(
        boost::bind(&LogRecord::RunWrite, this));
    this->startThreadCondition.wait(writeLock);
  }

  return true;
}

//////////////////////////////////////////////////
const std::string &LogRecord::GetEncoding() const
{
  return this->encoding;
}

//////////////////////////////////////////////////
void LogRecord::Fini()
{
  do
  {
    boost::mutex::scoped_lock lock(this->controlMutex);
    this->cleanupCondition.notify_all();
  } while (this->cleanupThread.joinable() &&
          !this->cleanupThread.timed_join(
            boost::posix_time::milliseconds(1000)));

  boost::mutex::scoped_lock lock(this->controlMutex);
  this->connections.clear();

  // Remove all the logs.
  this->ClearLogs();

  this->logControlSub.reset();
  this->logStatusPub.reset();
  this->node.reset();
}

//////////////////////////////////////////////////
void LogRecord::Stop()
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  this->running = false;
  this->cleanupCondition.notify_all();
}

//////////////////////////////////////////////////
void LogRecord::ClearLogs()
{
  boost::mutex::scoped_lock logLock(this->writeMutex);

  // Delete all the log objects
  for (Log_M::iterator iter = this->logs.begin();
      iter != this->logs.end(); ++iter)
  {
    delete iter->second;
  }

  this->logs.clear();
  this->logsEnd = this->logs.end();
}

//////////////////////////////////////////////////
void LogRecord::SetPaused(bool _paused)
{
  this->paused = _paused;
}

//////////////////////////////////////////////////
bool LogRecord::GetPaused() const
{
  return this->paused;
}

//////////////////////////////////////////////////
bool LogRecord::GetRunning() const
{
  return this->running;
}

//////////////////////////////////////////////////
void LogRecord::Add(const std::string &_name, const std::string &_filename,
                    boost::function<bool (std::ostringstream &)> _logCallback)
{
  boost::mutex::scoped_lock logLock(this->writeMutex);

  // Check to see if the log has already been added.
  if (this->logs.find(_name) != this->logs.end())
  {
    GZ_ASSERT(this->logs.find(_name)->second != NULL, "Unable to find log");

    if (this->logs.find(_name)->second->GetRelativeFilename() != _filename)
    {
      gzerr << "Attempting to add a duplicate log object named["
          << _name << "] with a filename of [" << _filename << "].\n";
      return;
    }
    else
    {
      return;
    }
  }

  LogRecord::Log *newLog;

  // Create a new log object
  try
  {
    newLog = new LogRecord::Log(this, _filename, _logCallback);
  }
  catch(...)
  {
    gzthrow("Unable to create log. File permissions are probably bad.");
  }

  if (this->running)
    newLog->Start(this->logCompletePath);

  // Add the log to our map
  this->logs[_name] = newLog;

  // Update the pointer to the end of the log objects list.
  this->logsEnd = this->logs.end();

  if (!this->node)
  {
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();

    this->logControlSub = this->node->Subscribe("~/log/control",
        &LogRecord::OnLogControl, this);
    this->logStatusPub = this->node->Advertise<msgs::LogStatus>("~/log/status");
  }
}

//////////////////////////////////////////////////
bool LogRecord::Remove(const std::string &_name)
{
  boost::mutex::scoped_lock logLock(this->writeMutex);

  bool result = false;

  Log_M::iterator iter = this->logs.find(_name);
  if (iter != this->logs.end())
  {
    delete iter->second;
    this->logs.erase(iter);

    // Update the pointer to the end of the log objects list.
    this->logsEnd = this->logs.end();

    result = true;
  }

  return result;
}

//////////////////////////////////////////////////
std::string LogRecord::GetFilename(const std::string &_name) const
{
  boost::mutex::scoped_lock logLock(this->writeMutex);

  std::string result;

  Log_M::const_iterator iter = this->logs.find(_name);
  if (iter != this->logs.end())
  {
    GZ_ASSERT(iter->second, "Invalid log");
    result = iter->second->GetCompleteFilename();
  }
  else
    result = this->logs.begin()->second->GetCompleteFilename();

  return result;
}

//////////////////////////////////////////////////
unsigned int LogRecord::GetFileSize(const std::string &_name) const
{
  unsigned int result = 0;

  // Get the filename of the specified log object;
  std::string filename = this->GetFilename(_name);

  // Get the size of the log file on disk.
  if (!filename.empty())
  {
    // Get the size of the file
    if (!filename.empty() && boost::filesystem::exists(filename))
      result = boost::filesystem::file_size(filename);
  }

  // Add in the contents of the write buffer. This is the data that will be
  // written to disk soon.
  {
    boost::mutex::scoped_lock lock(this->writeMutex);
    Log_M::const_iterator iter = this->logs.find(_name);

    if (iter != this->logs.end())
    {
      GZ_ASSERT(iter->second, "Log object is NULL");
      result += iter->second->GetBufferSize();
    }
  }

  return result;
}

//////////////////////////////////////////////////
void LogRecord::SetBasePath(const std::string &_path)
{
  // Make sure the  directory exists
  if (!boost::filesystem::exists(_path))
    boost::filesystem::create_directories(_path);

  // Make sure we have a directory
  if (!boost::filesystem::is_directory(_path))
  {
    gzerr << "Path " << _path << " is not a directory. Please only specify a "
           << "directory for data logging.\n";
    return;
  }

  // Make sure the path is writable.
  // Note: This is not cross-platform compatible.
  if (access(_path.c_str(), W_OK) != 0)
  {
    gzerr << "You do no have permission to write into " << _path << "\n";
    return;
  }

  this->logBasePath = _path;
}

//////////////////////////////////////////////////
std::string LogRecord::GetBasePath() const
{
  return this->logBasePath.string();
}

//////////////////////////////////////////////////
bool LogRecord::GetFirstUpdate() const
{
  return this->firstUpdate;
}

//////////////////////////////////////////////////
void LogRecord::Notify()
{
  if (this->running)
    this->updateCondition.notify_all();
}

//////////////////////////////////////////////////
void LogRecord::RunUpdate()
{
  boost::mutex::scoped_lock updateLock(this->updateMutex);
  this->startThreadCondition.notify_all();

  // This loop will write data to disk.
  while (!this->stopThread)
  {
    // Don't completely lock, just to be safe.
    this->updateCondition.wait(updateLock);

    if (!this->stopThread)
      this->Update();
  }
}

//////////////////////////////////////////////////
void LogRecord::Update()
{
  if (!this->paused)
  {
    unsigned int size = 0;

    {
      boost::mutex::scoped_lock lock(this->writeMutex);

      // Collect all the new log data. This will not write data to disk.
      for (this->updateIter = this->logs.begin();
           this->updateIter != this->logsEnd; ++this->updateIter)
      {
        size += this->updateIter->second->Update();
      }
    }

    if (this->firstUpdate)
    {
      this->firstUpdate = false;
      this->startTime = common::Time::GetWallTime();
    }

    // Signal that new data is available.
    if (size > 0)
      this->dataAvailableCondition.notify_one();

    this->currTime = common::Time::GetWallTime();

    // Output the new log status
    this->PublishLogStatus();
  }
}

//////////////////////////////////////////////////
void LogRecord::RunWrite()
{
  // Wait for new data.
  boost::mutex::scoped_lock lock(this->runWriteMutex);
  this->startThreadCondition.notify_all();

  // This loop will write data to disk.
  while (!this->stopThread)
  {
    this->dataAvailableCondition.wait(lock);

    this->Write(false);
  }
}

//////////////////////////////////////////////////
void LogRecord::Write(bool /*_force*/)
{
  boost::mutex::scoped_lock lock(this->writeMutex);

  // Collect all the new log data.
  for (this->updateIter = this->logs.begin();
      this->updateIter != this->logsEnd; ++this->updateIter)
  {
    this->updateIter->second->Write();
  }
}

//////////////////////////////////////////////////
common::Time LogRecord::GetRunTime() const
{
  return this->currTime - this->startTime;
}

//////////////////////////////////////////////////
LogRecord::Log::Log(LogRecord *_parent, const std::string &_relativeFilename,
                 boost::function<bool (std::ostringstream &)> _logCB)
{
  this->parent = _parent;
  this->logCB = _logCB;

  this->relativeFilename = _relativeFilename;
}

//////////////////////////////////////////////////
LogRecord::Log::~Log()
{
  this->Stop();
}

//////////////////////////////////////////////////
unsigned int LogRecord::Log::Update()
{
  std::ostringstream stream;

  // Get log data via the callback.
  if (this->logCB(stream))
  {
    std::string data = stream.str();
    if (!data.empty())
    {
      const std::string encodingLocal = this->parent->GetEncoding();

      this->buffer.append("<chunk encoding='");
      this->buffer.append(encodingLocal);
      this->buffer.append("'>\n");

      this->buffer.append("<![CDATA[");
      // Compress the data.
      if (encodingLocal == "bz2")
      {
        std::string str;

        // Compress to bzip2
        {
          boost::iostreams::filtering_ostream out;
          out.push(boost::iostreams::bzip2_compressor());
          out.push(std::back_inserter(str));
          boost::iostreams::copy(boost::make_iterator_range(data), out);
        }

        // Encode in base64.
        Base64Encode(str.c_str(), str.size(), this->buffer);
      }
      else if (encodingLocal == "zlib")
      {
        std::string str;

        // Compress to zlib
        {
          boost::iostreams::filtering_ostream out;
          out.push(boost::iostreams::zlib_compressor());
          out.push(std::back_inserter(str));
          boost::iostreams::copy(boost::make_iterator_range(data), out);
        }

        // Encode in base64.
        Base64Encode(str.c_str(), str.size(), this->buffer);
      }
      else if (encodingLocal == "txt")
        this->buffer.append(data);
      else
        gzerr << "Unknown log file encoding[" << encodingLocal << "]\n";
      this->buffer.append("]]>\n");

      this->buffer.append("</chunk>\n");
    }
  }

  return this->buffer.size();
}

//////////////////////////////////////////////////
void LogRecord::Log::ClearBuffer()
{
  this->buffer.clear();
}

//////////////////////////////////////////////////
unsigned int LogRecord::Log::GetBufferSize()
{
  return this->buffer.size();
}

//////////////////////////////////////////////////
std::string LogRecord::Log::GetRelativeFilename() const
{
  return this->relativeFilename;
}

//////////////////////////////////////////////////
std::string LogRecord::Log::GetCompleteFilename() const
{
  return this->completePath.string();
}

//////////////////////////////////////////////////
void LogRecord::Log::Stop()
{
  if (this->logFile.is_open())
  {
    this->Update();
    this->Write();

    std::string xmlEnd = "</gazebo_log>";
    this->logFile.write(xmlEnd.c_str(), xmlEnd.size());

    this->logFile.close();
  }

  this->completePath.clear();
}

//////////////////////////////////////////////////
void LogRecord::Log::Start(const boost::filesystem::path &_path)
{
  // Make the full path for the log file
  this->completePath = _path / this->relativeFilename;

  // Make sure the file does not exist
  if (boost::filesystem::exists(this->completePath))
    gzlog << "Filename[" + this->completePath.string() + "], already exists."
      << " The log file will be overwritten.\n";

  std::ostringstream stream;
  stream << "<?xml version='1.0'?>\n"
         << "<gazebo_log>\n"
         << "<header>\n"
         << "<log_version>" << GZ_LOG_VERSION << "</log_version>\n"
         << "<gazebo_version>" << GAZEBO_VERSION_FULL << "</gazebo_version>\n"
         << "<rand_seed>" << math::Rand::GetSeed() << "</rand_seed>\n"
         << "</header>\n";

  this->buffer.append(stream.str());
}

//////////////////////////////////////////////////
void LogRecord::Log::Write()
{
  // Make sure the file is open for writing
  if (!this->logFile.is_open())
  {
    // Try to open it...
    this->logFile.open(this->completePath.string().c_str(),
                       std::fstream::out | std::ios::binary);

    // Throw an error if we couldn't open the file for writing.
    if (!this->logFile.is_open())
      gzthrow("Unable to open file for logging:" +
              this->completePath.string() + "]");
  }

  // Check to see if the log file still exists on disk. This will catch the
  // case when someone deletes a log file while recording.
  if (!boost::filesystem::exists(this->completePath.string().c_str()))
  {
    gzerr << "Log file[" << this->completePath << "] no longer exists. "
          << "Unable to write log data.\n";

    // We have to clear the buffer, or else it may grow indefinitely.
    this->buffer.clear();
    return;
  }

  // Write out the contents of the buffer.
  this->logFile.write(this->buffer.c_str(), this->buffer.size());
  this->logFile.flush();

  // Clear the buffer.
  this->buffer.clear();
}

//////////////////////////////////////////////////
void LogRecord::OnLogControl(ConstLogControlPtr &_data)
{
  if (_data->has_base_path() && !_data->base_path().empty())
    this->SetBasePath(_data->base_path());

  std::string msgEncoding = "zlib";
  if (_data->has_encoding())
    msgEncoding = _data->encoding();

  if (_data->has_start() && _data->start())
  {
    this->Start(msgEncoding);
  }
  else if (_data->has_stop() && _data->stop())
  {
    this->Stop();
  }
  else if (_data->has_paused())
  {
    this->SetPaused(_data->paused());
  }

  // Output the new log status
  this->PublishLogStatus();
}

//////////////////////////////////////////////////
void LogRecord::PublishLogStatus()
{
  if (this->logs.empty() || !this->logStatusPub ||
      !this->logStatusPub->HasConnections())
    return;

  /// \todo right now this function will only report on the first log.

  msgs::LogStatus msg;
  unsigned int size = 0;

  // Set the time of the status message
  msgs::Set(msg.mutable_sim_time(), this->GetRunTime());

  // Set the log recording base path name
  msg.mutable_log_file()->set_base_path(this->GetBasePath());

  // Get the full name of the log file
  msg.mutable_log_file()->set_full_path(this->GetFilename());

  // Set the URI of th log file
  msg.mutable_log_file()->set_uri(transport::Connection::GetLocalHostname());

  // Get the size of the log file
  size = this->GetFileSize();

  if (size < 1000)
    msg.mutable_log_file()->set_size_units(msgs::LogStatus::LogFile::BYTES);
  else if (size < 1e6)
  {
    msg.mutable_log_file()->set_size(size / 1.0e3);
    msg.mutable_log_file()->set_size_units(msgs::LogStatus::LogFile::K_BYTES);
  }
  else if (size < 1e9)
  {
    msg.mutable_log_file()->set_size(size / 1.0e6);
    msg.mutable_log_file()->set_size_units(msgs::LogStatus::LogFile::M_BYTES);
  }
  else
  {
    msg.mutable_log_file()->set_size(size / 1.0e9);
    msg.mutable_log_file()->set_size_units(msgs::LogStatus::LogFile::G_BYTES);
  }

  this->logStatusPub->Publish(msg);
}

//////////////////////////////////////////////////
void LogRecord::Cleanup()
{
  boost::mutex::scoped_lock lock(this->controlMutex);
  this->startThreadCondition.notify_all();

  // Wait for the cleanup signal
  this->cleanupCondition.wait(lock);

  bool currentPauseState = this->pauseState;
  event::Events::pause(true);

  // Reset the flags
  this->paused = false;
  this->running = false;
  this->stopThread = true;

  // Kick the update thread
  {
    boost::mutex::scoped_lock updateLock(this->updateMutex);
    this->updateCondition.notify_all();
  }

  // Wait for the write thread, if it exists
  if (this->updateThread)
    this->updateThread->join();

  // Kick the write thread
  {
    boost::mutex::scoped_lock lock2(this->runWriteMutex);
    this->dataAvailableCondition.notify_all();
  }

  // Wait for the write thread, if it exists
  if (this->writeThread)
    this->writeThread->join();

  delete this->updateThread;
  this->updateThread = NULL;

  delete this->writeThread;
  this->writeThread = NULL;

  // Update and write one last time to make sure we log all data.
  this->Update();

  this->Write(true);

  // Stop all the logs
  for (Log_M::iterator iter = this->logs.begin();
      iter != this->logsEnd; ++iter)
  {
    iter->second->Stop();
  }

  // Reset the times
  this->startTime = this->currTime = common::Time();

  // Output the new log status
  this->PublishLogStatus();

  event::Events::pause(currentPauseState);
  this->readyToStart = true;
}

//////////////////////////////////////////////////
bool LogRecord::IsReadyToStart() const
{
  return this->readyToStart;
}

//////////////////////////////////////////////////
unsigned int LogRecord::GetBufferSize() const
{
  boost::mutex::scoped_lock lock(this->writeMutex);
  unsigned int size = 0;

  for (Log_M::const_iterator iter = this->logs.begin();
      iter != this->logs.end(); ++iter)
  {
    size += iter->second->GetBufferSize();
  }

  return size;
}
