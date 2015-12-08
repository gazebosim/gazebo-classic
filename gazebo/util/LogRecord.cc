/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>

  #include <io.h>

  // Seems like W_OK does not exists on Windows.
  // Reading access function documentation, the value should be 2
  // http://msdn.microsoft.com/en-us/library/1w06ktdy.aspx
  // Test for write permission.
  #define W_OK    2
  #define access _access
#endif

#include <functional>

#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/insert_linebreaks.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/archive/iterators/ostream_iterator.hpp>

#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <iomanip>

#include <ignition/math/Rand.hh>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Base64.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/LogRecordPrivate.hh"
#include "gazebo/util/LogRecord.hh"

using namespace gazebo;
using namespace util;

//////////////////////////////////////////////////
LogRecord::LogRecord()
: dataPtr(new LogRecordPrivate)
{
  this->dataPtr->pauseState = false;
  this->dataPtr->running = false;
  this->dataPtr->paused = false;
  this->dataPtr->initialized = false;
  this->dataPtr->stopThread = false;
  this->dataPtr->firstUpdate = true;
  this->dataPtr->readyToStart = false;

  // Get the user's home directory
#ifndef _WIN32
  const char *homePath = common::getEnv("HOME");
#else
  const char *homePath = common::getEnv("HOMEPATH");
#endif

  GZ_ASSERT(homePath, "HOME environment variable is missing");

  if (!homePath)
  {
    common::SystemPaths *paths = common::SystemPaths::Instance();
    this->dataPtr->logBasePath = paths->GetTmpPath() + "/gazebo";
  }
  else
    this->dataPtr->logBasePath = boost::filesystem::path(homePath);

  this->dataPtr->logBasePath /= "/.gazebo/log/";

  this->dataPtr->logsEnd = this->dataPtr->logs.end();

  this->dataPtr->connections.push_back(
     event::Events::ConnectPause(
       std::bind(&LogRecord::OnPause, this, std::placeholders::_1)));
}

//////////////////////////////////////////////////
void LogRecord::OnPause(const bool _pause)
{
  this->dataPtr->pauseState = _pause;
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

  this->dataPtr->logSubDir = _subdir;

  this->ClearLogs();

  this->dataPtr->initialized = true;
  this->dataPtr->running = false;
  this->dataPtr->paused = false;
  this->dataPtr->stopThread = false;
  this->dataPtr->firstUpdate = true;
  this->dataPtr->readyToStart = true;

  return true;
}

//////////////////////////////////////////////////
bool LogRecord::Start(const std::string &_encoding, const std::string &_path)
{
  std::unique_lock<std::mutex> lock(this->dataPtr->controlMutex);

  // Make sure ::Init has been called.
  if (!this->dataPtr->initialized)
  {
    gzerr << "LogRecord has not been initialized." << std::endl;
    return false;
  }

  // Check to see if the logger is already started.
  if (this->dataPtr->running)
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
    this->dataPtr->logBasePath = boost::filesystem::path(_path);
    this->dataPtr->logCompletePath = this->dataPtr->logBasePath;
  }
  else
  {
    this->dataPtr->logCompletePath = this->dataPtr->logBasePath /
      logTimeDir / this->dataPtr->logSubDir;
  }

  // Create the log directory if necessary
  if (!boost::filesystem::exists(this->dataPtr->logCompletePath))
    boost::filesystem::create_directories(this->dataPtr->logCompletePath);

  if (_encoding != "bz2" && _encoding != "txt" && _encoding != "zlib")
    gzthrow("Invalid log encoding[" + _encoding +
            "]. Must be one of [bz2, zlib, txt]");

  this->dataPtr->encoding = _encoding;

  {
    std::unique_lock<std::mutex> logLock(this->dataPtr->writeMutex);
    this->dataPtr->logsEnd = this->dataPtr->logs.end();

    // Start all the logs
    for (LogRecordPrivate::Log_M::iterator iter = this->dataPtr->logs.begin();
         iter != this->dataPtr->logsEnd; ++iter)
      iter->second->Start(this->dataPtr->logCompletePath);
  }

  this->dataPtr->running = true;
  this->dataPtr->paused = false;
  this->dataPtr->firstUpdate = true;
  this->dataPtr->stopThread = false;
  this->dataPtr->readyToStart = false;

  this->dataPtr->startTime = this->dataPtr->currTime = common::Time();

  // Create a thread to cleanup recording.
  this->dataPtr->cleanupThread = std::thread(
      std::bind(&LogRecord::Cleanup, this));
  // Wait for thread to start
  this->dataPtr->startThreadCondition.wait(lock);

  // Start the update thread if it has not already been started
  if (!this->dataPtr->updateThread)
  {
    std::unique_lock<std::mutex> updateLock(this->dataPtr->updateMutex);
    this->dataPtr->updateThread = new std::thread(
        std::bind(&LogRecord::RunUpdate, this));
    this->dataPtr->startThreadCondition.wait(updateLock);
  }

  // Start the writing thread if it has not already been started
  if (!this->dataPtr->writeThread)
  {
    std::unique_lock<std::mutex> writeLock(this->dataPtr->runWriteMutex);
    this->dataPtr->writeThread = new std::thread(
        std::bind(&LogRecord::RunWrite, this));
    this->dataPtr->startThreadCondition.wait(writeLock);
  }

  return true;
}

//////////////////////////////////////////////////
const std::string &LogRecord::GetEncoding() const
{
  return this->Encoding();
}

//////////////////////////////////////////////////
const std::string &LogRecord::Encoding() const
{
  return this->dataPtr->encoding;
}

//////////////////////////////////////////////////
void LogRecord::Fini()
{
  {
    std::unique_lock<std::mutex> lock(this->dataPtr->controlMutex);
    this->dataPtr->cleanupCondition.notify_all();
  }

  this->dataPtr->cleanupThread.join();

  std::lock_guard<std::mutex> lock(this->dataPtr->controlMutex);
  this->dataPtr->connections.clear();

  // Remove all the logs.
  this->ClearLogs();

  this->dataPtr->logControlSub.reset();
  this->dataPtr->logStatusPub.reset();
  this->dataPtr->node.reset();
}

//////////////////////////////////////////////////
void LogRecord::Stop()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->controlMutex);

  this->dataPtr->running = false;
  this->dataPtr->cleanupCondition.notify_all();
}

//////////////////////////////////////////////////
void LogRecord::ClearLogs()
{
  std::lock_guard<std::mutex> logLock(this->dataPtr->writeMutex);

  // Delete all the log objects
  for (LogRecordPrivate::Log_M::iterator iter = this->dataPtr->logs.begin();
      iter != this->dataPtr->logs.end(); ++iter)
  {
    delete iter->second;
  }

  this->dataPtr->logs.clear();
  this->dataPtr->logsEnd = this->dataPtr->logs.end();
}

//////////////////////////////////////////////////
void LogRecord::SetPaused(const bool _paused)
{
  this->dataPtr->paused = _paused;
}

//////////////////////////////////////////////////
bool LogRecord::GetPaused() const
{
  return this->Paused();
}

//////////////////////////////////////////////////
bool LogRecord::Paused() const
{
  return this->dataPtr->paused;
}

//////////////////////////////////////////////////
bool LogRecord::GetRunning() const
{
  return this->Running();
}

//////////////////////////////////////////////////
bool LogRecord::Running() const
{
  return this->dataPtr->running;
}

//////////////////////////////////////////////////
void LogRecord::Add(const std::string &_name, const std::string &_filename,
                    std::function<bool (std::ostringstream &)> _logCallback)
{
  std::lock_guard<std::mutex> logLock(this->dataPtr->writeMutex);

  // Check to see if the log has already been added.
  if (this->dataPtr->logs.find(_name) != this->dataPtr->logs.end())
  {
    GZ_ASSERT(this->dataPtr->logs.find(_name)->second != NULL,
        "Unable to find log");

    if (this->dataPtr->logs.find(_name)->second->RelativeFilename() !=
        _filename)
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

  LogRecordPrivate::Log *newLog;

  // Create a new log object
  try
  {
    newLog = new LogRecordPrivate::Log(this, _filename, _logCallback);
  }
  catch(...)
  {
    gzthrow("Unable to create log. File permissions are probably bad.");
  }

  if (this->dataPtr->running)
    newLog->Start(this->dataPtr->logCompletePath);

  // Add the log to our map
  this->dataPtr->logs[_name] = newLog;

  // Update the pointer to the end of the log objects list.
  this->dataPtr->logsEnd = this->dataPtr->logs.end();

  if (!this->dataPtr->node)
  {
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init();

    this->dataPtr->logControlSub =
      this->dataPtr->node->Subscribe("~/log/control",
          &LogRecord::OnLogControl, this);
    this->dataPtr->logStatusPub =
      this->dataPtr->node->Advertise<msgs::LogStatus>("~/log/status");
  }
}

//////////////////////////////////////////////////
bool LogRecord::Remove(const std::string &_name)
{
  std::lock_guard<std::mutex> logLock(this->dataPtr->writeMutex);

  bool result = false;

  LogRecordPrivate::Log_M::iterator iter = this->dataPtr->logs.find(_name);
  if (iter != this->dataPtr->logs.end())
  {
    delete iter->second;
    this->dataPtr->logs.erase(iter);

    // Update the pointer to the end of the log objects list.
    this->dataPtr->logsEnd = this->dataPtr->logs.end();

    result = true;
  }

  return result;
}

//////////////////////////////////////////////////
std::string LogRecord::GetFilename(const std::string &_name) const
{
  return this->Filename(_name);
}

//////////////////////////////////////////////////
std::string LogRecord::Filename(const std::string &_name) const
{
  std::lock_guard<std::mutex> logLock(this->dataPtr->writeMutex);

  std::string result;

  LogRecordPrivate::Log_M::const_iterator iter =
    this->dataPtr->logs.find(_name);
  if (iter != this->dataPtr->logs.end())
  {
    GZ_ASSERT(iter->second, "Invalid log");
    result = iter->second->CompleteFilename();
  }
  else
    result = this->dataPtr->logs.begin()->second->CompleteFilename();

  return result;
}

//////////////////////////////////////////////////
unsigned int LogRecord::GetFileSize(const std::string &_name) const
{
  return this->FileSize(_name);
}

//////////////////////////////////////////////////
unsigned int LogRecord::FileSize(const std::string &_name) const
{
  unsigned int result = 0;

  // Get the filename of the specified log object;
  std::string filename = this->Filename(_name);

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
    std::lock_guard<std::mutex> lock(this->dataPtr->writeMutex);
    LogRecordPrivate::Log_M::const_iterator iter =
      this->dataPtr->logs.find(_name);

    if (iter != this->dataPtr->logs.end())
    {
      GZ_ASSERT(iter->second, "Log object is NULL");
      result += iter->second->BufferSize();
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

  this->dataPtr->logBasePath = _path;
}

//////////////////////////////////////////////////
std::string LogRecord::GetBasePath() const
{
  return this->BasePath();
}

//////////////////////////////////////////////////
std::string LogRecord::BasePath() const
{
  return this->dataPtr->logBasePath.string();
}

//////////////////////////////////////////////////
bool LogRecord::GetFirstUpdate() const
{
  return this->FirstUpdate();
}

//////////////////////////////////////////////////
bool LogRecord::FirstUpdate() const
{
  return this->dataPtr->firstUpdate;
}

//////////////////////////////////////////////////
void LogRecord::Notify()
{
  if (this->dataPtr->running)
    this->dataPtr->updateCondition.notify_all();
}

//////////////////////////////////////////////////
void LogRecord::RunUpdate()
{
  std::unique_lock<std::mutex> updateLock(this->dataPtr->updateMutex);
  this->dataPtr->startThreadCondition.notify_all();

  // This loop will write data to disk.
  while (!this->dataPtr->stopThread)
  {
    // Don't completely lock, just to be safe.
    this->dataPtr->updateCondition.wait(updateLock);

    if (!this->dataPtr->stopThread)
      this->Update();
  }
}

//////////////////////////////////////////////////
void LogRecord::Update()
{
  if (!this->dataPtr->paused)
  {
    unsigned int size = 0;

    {
      std::lock_guard<std::mutex> lock(this->dataPtr->writeMutex);

      // Collect all the new log data. This will not write data to disk.
      for (this->dataPtr->updateIter = this->dataPtr->logs.begin();
           this->dataPtr->updateIter != this->dataPtr->logsEnd;
           ++this->dataPtr->updateIter)
      {
        size += this->dataPtr->updateIter->second->Update();
      }
    }

    if (this->dataPtr->firstUpdate)
    {
      this->dataPtr->firstUpdate = false;
      this->dataPtr->startTime = common::Time::GetWallTime();
    }

    // Signal that new data is available.
    if (size > 0)
      this->dataPtr->dataAvailableCondition.notify_one();

    this->dataPtr->currTime = common::Time::GetWallTime();

    // Output the new log status
    this->PublishLogStatus();
  }
}

//////////////////////////////////////////////////
void LogRecord::RunWrite()
{
  // Wait for new data.
  std::unique_lock<std::mutex> lock(this->dataPtr->runWriteMutex);
  this->dataPtr->startThreadCondition.notify_all();

  // This loop will write data to disk.
  while (!this->dataPtr->stopThread)
  {
    this->dataPtr->dataAvailableCondition.wait(lock);

    this->Write(false);
  }
}

//////////////////////////////////////////////////
void LogRecord::Write(const bool /*_force*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->writeMutex);

  // Collect all the new log data.
  for (this->dataPtr->updateIter = this->dataPtr->logs.begin();
      this->dataPtr->updateIter != this->dataPtr->logsEnd;
      ++this->dataPtr->updateIter)
  {
    this->dataPtr->updateIter->second->Write();
  }
}

//////////////////////////////////////////////////
common::Time LogRecord::GetRunTime() const
{
  return this->RunTime();
}

//////////////////////////////////////////////////
common::Time LogRecord::RunTime() const
{
  return this->dataPtr->currTime - this->dataPtr->startTime;
}

//////////////////////////////////////////////////
LogRecordPrivate::Log::Log(LogRecord *_parent,
    const std::string &_relativeFilename,
    std::function<bool (std::ostringstream &)> _logCB)
{
  this->parent = _parent;
  this->logCB = _logCB;

  this->relativeFilename = _relativeFilename;
}

//////////////////////////////////////////////////
LogRecordPrivate::Log::~Log()
{
  this->Stop();
}

//////////////////////////////////////////////////
unsigned int LogRecordPrivate::Log::Update()
{
  std::ostringstream stream;

  // Get log data via the callback.
  if (this->logCB(stream))
  {
    std::string data = stream.str();
    if (!data.empty())
    {
      const std::string &encodingLocal = this->parent->Encoding();

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
void LogRecordPrivate::Log::ClearBuffer()
{
  this->buffer.clear();
}

//////////////////////////////////////////////////
unsigned int LogRecordPrivate::Log::BufferSize()
{
  return this->buffer.size();
}

//////////////////////////////////////////////////
std::string LogRecordPrivate::Log::RelativeFilename() const
{
  return this->relativeFilename;
}

//////////////////////////////////////////////////
std::string LogRecordPrivate::Log::CompleteFilename() const
{
  return this->completePath.string();
}

//////////////////////////////////////////////////
void LogRecordPrivate::Log::Stop()
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
void LogRecordPrivate::Log::Start(const boost::filesystem::path &_path)
{
  // Make the full path for the log file
  this->completePath = _path / this->relativeFilename;

  // Make sure the file does not exist
  if (boost::filesystem::exists(this->completePath))
    gzlog << "Filename [" + this->completePath.string() + "], already exists."
          << " The log file will be overwritten.\n";

  std::ostringstream stream;
  stream << "<?xml version='1.0'?>\n"
         << "<gazebo_log>\n"
         << "<header>\n"
         << "<log_version>" << GZ_LOG_VERSION << "</log_version>\n"
         << "<gazebo_version>" << GAZEBO_VERSION_FULL << "</gazebo_version>\n"
         << "<rand_seed>" << ignition::math::Rand::Seed() << "</rand_seed>\n"
         << "</header>\n";

  this->buffer.append(stream.str());
}

//////////////////////////////////////////////////
void LogRecordPrivate::Log::Write()
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
  if (this->dataPtr->logs.empty() || !this->dataPtr->logStatusPub ||
      !this->dataPtr->logStatusPub->HasConnections())
    return;

  /// \todo right now this function will only report on the first log.

  msgs::LogStatus msg;
  unsigned int size = 0;

  // Set the time of the status message
  msgs::Set(msg.mutable_sim_time(), this->RunTime());

  // Set the log recording base path name
  msg.mutable_log_file()->set_base_path(this->BasePath());

  // Get the full name of the log file
  msg.mutable_log_file()->set_full_path(this->Filename());

  // Set the URI of th log file
  msg.mutable_log_file()->set_uri(transport::Connection::GetLocalHostname());

  // Get the size of the log file
  size = this->FileSize();

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

  this->dataPtr->logStatusPub->Publish(msg);
}

//////////////////////////////////////////////////
void LogRecord::Cleanup()
{
  std::unique_lock<std::mutex> lock(this->dataPtr->controlMutex);
  this->dataPtr->startThreadCondition.notify_all();

  // Wait for the cleanup signal
  this->dataPtr->cleanupCondition.wait(lock);

  bool currentPauseState = this->dataPtr->pauseState;
  event::Events::pause(true);

  // Reset the flags
  this->dataPtr->paused = false;
  this->dataPtr->running = false;
  this->dataPtr->stopThread = true;

  // Kick the update thread
  {
    std::lock_guard<std::mutex> updateLock(this->dataPtr->updateMutex);
    this->dataPtr->updateCondition.notify_all();
  }

  // Wait for the write thread, if it exists
  if (this->dataPtr->updateThread)
    this->dataPtr->updateThread->join();

  // Kick the write thread
  {
    std::lock_guard<std::mutex> lock2(this->dataPtr->runWriteMutex);
    this->dataPtr->dataAvailableCondition.notify_all();
  }

  // Wait for the write thread, if it exists
  if (this->dataPtr->writeThread)
    this->dataPtr->writeThread->join();

  delete this->dataPtr->updateThread;
  this->dataPtr->updateThread = NULL;

  delete this->dataPtr->writeThread;
  this->dataPtr->writeThread = NULL;

  // Update and write one last time to make sure we log all data.
  this->Update();

  this->Write(true);

  // Stop all the logs
  for (LogRecordPrivate::Log_M::iterator iter = this->dataPtr->logs.begin();
      iter != this->dataPtr->logsEnd; ++iter)
  {
    iter->second->Stop();
  }

  // Reset the times
  this->dataPtr->startTime = this->dataPtr->currTime = common::Time();

  // Output the new log status
  this->PublishLogStatus();

  event::Events::pause(currentPauseState);
  this->dataPtr->readyToStart = true;
}

//////////////////////////////////////////////////
bool LogRecord::IsReadyToStart() const
{
  return this->dataPtr->readyToStart;
}

//////////////////////////////////////////////////
unsigned int LogRecord::GetBufferSize() const
{
  return this->BufferSize();
}

//////////////////////////////////////////////////
unsigned int LogRecord::BufferSize() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->writeMutex);
  unsigned int size = 0;

  for (LogRecordPrivate::Log_M::const_iterator iter =
       this->dataPtr->logs.begin();
       iter != this->dataPtr->logs.end(); ++iter)
  {
    size += iter->second->BufferSize();
  }

  return size;
}
