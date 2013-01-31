/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/date_time.hpp>

#include "gazebo/math/Rand.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/LogRecord.hh"

#include "gazebo/gazebo_config.h"

using namespace gazebo;
using namespace common;

/// Convert binary values to base64 characters
typedef boost::archive::iterators::base64_from_binary<
        // retrieve 6 bit integers from a sequence of 8 bit bytes
        boost::archive::iterators::transform_width<const char *, 6, 8> >
        Base64Text;

//////////////////////////////////////////////////
LogRecord::LogRecord()
{
  this->running = false;
  this->paused = false;
  this->initialized = false;
  this->stopThread = false;
  this->firstUpdate = true;

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

  return true;
}

//////////////////////////////////////////////////
bool LogRecord::Start(const std::string &_encoding)
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

  this->logCompletePath = this->logBasePath / logTimeDir / this->logSubDir;

  // Create the log directory if necessary
  if (!boost::filesystem::exists(this->logCompletePath))
    boost::filesystem::create_directories(logCompletePath);

  if (_encoding != "bz2" && _encoding != "txt")
    gzthrow("Invalid log encoding[" + _encoding +
            "]. Must be one of [bz2, txt]");

  this->encoding = _encoding;

  {
    boost::mutex::scoped_lock logLock(this->writeMutex);
    this->logsEnd = this->logs.end();

    // Start all the logs
    for (Log_M::iterator iter = this->logs.begin();
         iter != this->logsEnd; ++iter)
      iter->second->Start(logCompletePath);
  }

  // Listen to the world update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&LogRecord::Update, this, _1));

  // Start the writing thread if it has not already been started
  if (!this->writeThread)
    this->writeThread = new boost::thread(boost::bind(&LogRecord::Run, this));

  this->running = true;
  this->paused = false;
  this->firstUpdate = true;

  this->startTime = this->currTime = common::Time();

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
  this->stopThread = true;

  this->Stop();

  // Wait for the write thread, if it exists
  if (this->writeThread)
    this->writeThread->join();
  delete this->writeThread;
  this->writeThread = NULL;
}

//////////////////////////////////////////////////
void LogRecord::Stop()
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  // Disconnect from the world update signale
  if (this->updateConnection)
    event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->updateConnection.reset();

  // Kick the write thread
  this->dataAvailableCondition.notify_one();

  // Remove all the logs.
  this->ClearLogs();

  // Reset the times
  this->startTime = this->currTime = common::Time();

  // Reset the flags
  this->running = false;
  this->paused = false;
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
    /// \todo Good place to use GZ_ASSERT
    /// GZ_ASSERT(this->logs.find(_name)->second != NULL);

    if (this->logs.find(_name)->second->GetRelativeFilename() != _filename)
    {
      gzthrow(std::string("Attempting to add a duplicate log object named[")
          + _name + "] with a filename of [" + _filename + "]\n");
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
    /// \TODO GZ_ASSERT(iter->second);
    result = iter->second->GetCompleteFilename();
  }

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
  {
    gzerr << "Log directory[" << _path << "] does not exist.\n";
    return;
  }

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
void LogRecord::Update(const common::UpdateInfo &_info)
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
      this->startTime = _info.simTime;
    }


    // Signal that new data is available.
    if (size > 0)
      this->dataAvailableCondition.notify_one();

    this->currTime = _info.simTime;
  }
}

//////////////////////////////////////////////////
void LogRecord::Run()
{
  if (!this->running)
    gzerr << "Running LogRecord before it has been started." << std::endl;

  this->stopThread = false;

  // This loop will write data to disk.
  while (!this->stopThread)
  {
    {
      // Wait for new data.
      boost::mutex::scoped_lock lock(this->writeMutex);
      this->dataAvailableCondition.wait(lock);

      // Collect all the new log data.
      for (this->updateIter = this->logs.begin();
           this->updateIter != this->logsEnd; ++this->updateIter)
      {
        this->updateIter->second->Write();
      }
    }

    // Throttle the write loop.
    common::Time::MSleep(1000);
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
LogRecord::Log::~Log()
{
  std::string xmlEnd = "</gazebo_log>";
  this->logFile.write(xmlEnd.c_str(), xmlEnd.size());

  this->logFile.close();
}

//////////////////////////////////////////////////
unsigned int LogRecord::Log::Update()
{
  std::ostringstream stream;

  // Get log data via the callback.
  if (this->logCB(stream) && !stream.str().empty())
  {
    const std::string encoding = this->parent->GetEncoding();

    this->buffer.append("<chunk encoding='");
    this->buffer.append(encoding);
    this->buffer.append("'>\n");

    this->buffer.append("<![CDATA[");
    {
      // Compress the data.
      if (encoding == "bz2")
      {
        std::string str;

        // Compress to bzip2
        {
          boost::iostreams::filtering_ostream out;
          out.push(boost::iostreams::bzip2_compressor());
          out.push(std::back_inserter(str));
          out << stream.str();
          out.flush();
        }

        // Encode in base64.
        std::copy(Base64Text(str.c_str()),
                  Base64Text(str.c_str() + str.size()),
                  std::back_inserter(this->buffer));
      }
      else if (encoding == "txt")
        this->buffer.append(stream.str());
      else
        gzerr << "Unknown log file encoding[" << encoding << "]\n";
    }
    this->buffer.append("]]>\n");

    this->buffer.append("</chunk>\n");
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
void LogRecord::Log::Start(const boost::filesystem::path &_path)
{
  // Make the full path for the log file
  this->completePath = _path / this->relativeFilename;

  // Make sure the file does not exist
  if (boost::filesystem::exists(this->completePath))
    gzthrow("Filename[" + this->completePath.string() + "], already exists\n");
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
