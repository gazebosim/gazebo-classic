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
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/date_time.hpp>

#include "gazebo/math/Rand.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Logger.hh"

#include "gazebo/gazebo_config.h"

using namespace gazebo;
using namespace common;

/// Convert binary values to base64 characters
typedef boost::archive::iterators::base64_from_binary<
        // retrieve 6 bit integers from a sequence of 8 bit bytes
        boost::archive::iterators::transform_width<const char *, 6, 8> >
        Base64Text;

//////////////////////////////////////////////////
Logger::Logger()
{
  this->stop = true;

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

  this->logsEnd = this->logs.end();
}

//////////////////////////////////////////////////
Logger::~Logger()
{
  // Stop the write thread.
  this->Stop();

  // Delete all the log objects
  for (Log_M::iterator iter = this->logs.begin();
       iter != this->logs.end(); ++iter)
  {
    delete iter->second;
  }

  this->logs.clear();
}

//////////////////////////////////////////////////
bool Logger::Init(const std::string &_subdir)
{
  if (!_subdir.empty())
    this->logPathname += "/" + _subdir;

  this->logsEnd = this->logs.end();
  return true;
}


//////////////////////////////////////////////////
void Logger::Start()
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  // Check to see if the logger is already started.
  if (!this->stop)
    return;

  // Create the log directory if necessary
  boost::filesystem::path path(this->logPathname);
  if (!boost::filesystem::exists(path))
    boost::filesystem::create_directories(path);

  this->logsEnd = this->logs.end();

  this->stop = false;

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
    return;
  }

  // Start the logging thread
  if (!this->writeThread)
    this->writeThread = new boost::thread(boost::bind(&Logger::Run, this));
  else
  {
    gzerr << "Logger has already been initialized\n";
    return;
  }
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
void Logger::Add(const std::string &_name, const std::string &_filename,
                 boost::function<bool (std::ostringstream &)> _logCallback)
{
  boost::mutex::scoped_lock lock(this->controlMutex);

  // Check to see if the logger is already started.
  if (this->stop)
    return;

  if (this->logs.find(_name) != this->logs.end())
    gzthrow("Log file with name[" + _name + "] already exists.\n");

  // Make the full path
  boost::filesystem::path path = boost::filesystem::path(this->logPathname);
  path = boost::filesystem::operator/(path, _filename);

  // Make sure the file does not exist
  if (boost::filesystem::exists(path))
    gzthrow("Filename[" + path.string() + "], already exists\n");

  Logger::Log *newLog;

  // Create a new log object
  try
  {
    newLog = new Logger::Log(path.string(), _logCallback);
  }
  catch(...)
  {
    gzthrow("Unable to create log. File permissions are probably bad.");
  }

  // Add the log to our map
  this->logs[_name] = newLog;

  // Update the pointer to the end of the log objects list.
  this->logsEnd = this->logs.end();
}

//////////////////////////////////////////////////
bool Logger::Remove(const std::string &_name)
{
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
void Logger::Update()
{
  {
    boost::mutex::scoped_lock lock(this->writeMutex);

    // Collect all the new log data. This will not write data to disk.
    for (this->updateIter = this->logs.begin();
         this->updateIter != this->logsEnd; ++this->updateIter)
    {
      this->updateIter->second->Update();
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

      // Collect all the new log data. This will not write data to disk.
      for (this->updateIter = this->logs.begin();
           this->updateIter != this->logsEnd; ++this->updateIter)
      {
        this->updateIter->second->Write();
      }
    }

    // Throttle the write loop.
    common::Time::MSleep(2000);
  }
}

//////////////////////////////////////////////////
Logger::Log::Log(const std::string &_filename,
                 boost::function<bool (std::ostringstream &)> _logCB)
{
  this->logCB = _logCB;

  this->filename = _filename;
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
Logger::Log::~Log()
{
  std::string xmlEnd = "</gazebo_log>";
  this->logFile.write(xmlEnd.c_str(), xmlEnd.size());

  this->logFile.close();
}

//////////////////////////////////////////////////
void Logger::Log::Update()
{
  std::ostringstream stream;

  if (this->logCB(stream) && !stream.str().empty())
  {
    std::string encoding = "bz2";

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
      else
        this->buffer.append(stream.str());
    }
    this->buffer.append("]]>\n");

    this->buffer.append("</chunk>\n");
  }
}

//////////////////////////////////////////////////
void Logger::Log::ClearBuffer()
{
  this->buffer.clear();
}

//////////////////////////////////////////////////
void Logger::Log::Write()
{
  // Make sure the file is open for writing
  if (!this->logFile.is_open())
  {
    // Try to open it...
    this->logFile.open(this->filename.c_str(),
                       std::fstream::out | std::ios::binary);

    // Throw an error if we couldn't open the file for writing.
    if (!this->logFile.is_open())
      gzthrow("Unable to open file for logging:" + this->filename + "]");
  }

  // Write out the contents of the buffer.
  this->logFile.write(this->buffer.c_str(), this->buffer.size());
  this->logFile.flush();

  // Clear the buffer.
  this->buffer.clear();
}
