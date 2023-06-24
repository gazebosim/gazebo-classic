/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <string>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <sstream>

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/CommonIface.hh"

#include "gazebo/gazebo_config.h"

using namespace gazebo;
using namespace common;

FileLogger gazebo::common::Console::log("");
Logger Console::msg("[Msg] ", 32, Logger::STDOUT);
Logger Console::err("[Err] ", 31, Logger::STDERR);
Logger Console::dbg("[Dbg] ", 36, Logger::STDOUT);
Logger Console::warn("[Wrn] ", 33, Logger::STDERR);

bool Console::quiet = true;

//////////////////////////////////////////////////
void Console::SetQuiet(bool _quiet)
{
  quiet = _quiet;
}

//////////////////////////////////////////////////
bool Console::GetQuiet()
{
  return quiet;
}

/////////////////////////////////////////////////
Logger::Logger(const std::string &_prefix, int _color, LogType _type)
  : std::ostream(new Buffer(_type, _color)), color(_color), prefix(_prefix)
{
  this->setf(std::ios_base::unitbuf);
}

/////////////////////////////////////////////////
Logger::~Logger()
{
  delete this->rdbuf();
}

/////////////////////////////////////////////////
Logger &Logger::operator()()
{
  Console::log << "(" << Time::GetWallTime() << ") ";
  (*this) << this->prefix;

  return (*this);
}

/////////////////////////////////////////////////
Logger &Logger::operator()(const std::string &_file, int _line)
{
  int index = _file.find_last_of("/") + 1;

  Console::log << "(" << Time::GetWallTime() << ") ";
  std::stringstream prefixString;
  prefixString << this->prefix
    << "[" << _file.substr(index , _file.size() - index) << ":"
    << _line << "] ";
  (*this) << prefixString.str();

  return (*this);
}

/////////////////////////////////////////////////
Logger::Buffer::Buffer(LogType _type, int _color)
  :  type(_type), color(_color)
{
}

/////////////////////////////////////////////////
Logger::Buffer::~Buffer()
{
  // Can't throw from a destructor
  try
  {
    this->pubsync();
  }
  catch(...)
  {
    std::cerr << "Exception thrown while pubsync'ing Buffer" << std::endl;
  }
}

/////////////////////////////////////////////////
int Logger::Buffer::sync()
{
  // Log messages to disk
  Console::log << this->str();
  Console::log.flush();

  // Output to terminal
  if (!Console::GetQuiet())
  {
    if (this->type == Logger::STDOUT)
    {
      #ifndef _WIN32
      std::cout << "\033[1;" << this->color << "m" << this->str() << "\033[0m";
      #else
      std::cout << this->str();
      #endif
    }
    else
    {
      #ifndef _WIN32
      std::cerr << "\033[1;" << this->color << "m" << this->str() << "\033[0m";
      #else
      std::cerr << this->str();
      #endif
    }
  }

  this->str("");
  return 0;
}

/////////////////////////////////////////////////
FileLogger::FileLogger(const std::string &_filename)
  : std::ostream(new Buffer(_filename)),
    logDirectory("")
{
  this->setf(std::ios_base::unitbuf);
}

/////////////////////////////////////////////////
FileLogger::~FileLogger()
{
  delete this->rdbuf();
}

/////////////////////////////////////////////////
void FileLogger::Init(const std::string &_prefix, const std::string &_filename)
{
  if (!getenv(HOMEDIR))
  {
    gzerr << "Missing " << HOMEDIR << " environment variable."
          << "No log file will be generated.";
    return;
  }

  FileLogger::Buffer *buf = static_cast<FileLogger::Buffer*>(
      this->rdbuf());

  boost::filesystem::path logPath(getenv(HOMEDIR));

  // Create a subdirectory for the informational log. The name of the directory
  // will be <PREFIX><MASTER_PORT>. E.g.: server-11346. If the environment
  // variable GAZEBO_MASTER_URI is not present or invalid, <MASTER_PORT> will
  // be replaced by "default".
  boost::filesystem::path subdir(_prefix + FileLogger::GetMasterPort());
  logPath = logPath / ".gazebo/" / subdir;

  // Create the log directory if it doesn't exist.
  if (!boost::filesystem::exists(logPath))
    boost::filesystem::create_directories(logPath);

  logPath /= _filename;

  // Check if the Init method has been already called, and if so
  // remove current buffer.
  if (buf->stream && buf->stream->is_open())
  {
    buf->stream->flush();
    buf->stream->close();
  }

  // If the logPath is a directory, just rename it.
  if (boost::filesystem::is_directory(logPath))
  {
    std::string newPath = logPath.string() + ".old";
    boost::system::error_code ec;
    boost::filesystem::rename(logPath, newPath, ec);
    if (ec == boost::system::errc::success)
      std::cerr << "Existing log directory [" << logPath
                << "] renamed to [" << newPath << "]" << std::endl;
    else
    {
      std::cerr << "Unable to rename existing log directory [" << logPath
                << "] to [" << newPath << "]. Reason: " << ec.message();
      return;
    }
  }

  if (!buf->stream)
    buf->stream = new std::ofstream(logPath.string().c_str(), std::ios::out);
  else
    buf->stream->open(logPath.string().c_str(), std::ios::out);

  if (!buf->stream->is_open())
    std::cerr << "Error opening log file: " << logPath << std::endl;

  // Output the version of gazebo.
  (*buf->stream) << GAZEBO_VERSION_HEADER << std::endl;

  // Update the log directory name.
  if (boost::filesystem::is_directory(logPath))
    this->logDirectory = logPath.string();
  else
    this->logDirectory = logPath.branch_path().string();
}

/////////////////////////////////////////////////
FileLogger &FileLogger::operator()()
{
  (*this) << "(" << Time::GetWallTime() << ") ";
  return (*this);
}

/////////////////////////////////////////////////
FileLogger &FileLogger::operator()(const std::string &_file, int _line)
{
  int index = _file.find_last_of("/") + 1;
  (*this) << "(" << Time::GetWallTime() << ") ["
    << _file.substr(index , _file.size() - index) << ":" << _line << "]";

  return (*this);
}

/////////////////////////////////////////////////
std::string FileLogger::GetMasterPort()
{
  char *charURI = getenv("GAZEBO_MASTER_URI");

  // Set to default port.
  if (charURI && strlen(charURI) > 0)
  {
    std::string masterURI = charURI;
    size_t lastColon = masterURI.find_last_of(":");
    if (lastColon != std::string::npos && lastColon != masterURI.size() - 1)
      return masterURI.substr(lastColon + 1, std::string::npos);
  }

  return boost::lexical_cast<std::string>(GAZEBO_DEFAULT_MASTER_PORT);
}

/////////////////////////////////////////////////
std::string FileLogger::GetLogDirectory() const
{
  return this->logDirectory;
}

/////////////////////////////////////////////////
FileLogger::Buffer::Buffer(const std::string &_filename)
  : stream(nullptr)
{
  if (!_filename.empty())
  {
    this->stream = new std::ofstream(_filename.c_str(), std::ios::out);
  }
}

/////////////////////////////////////////////////
FileLogger::Buffer::~Buffer()
{
  try
  {
    if (this->stream)
      static_cast<std::ofstream*>(this->stream)->close();
    delete this->stream;
  }
  catch(...)
  {
    std::cerr << "Exception thrown while closing Buffer" << std::endl;
  }
}

/////////////////////////////////////////////////
int FileLogger::Buffer::sync()
{
  if (!this->stream)
    return -1;

  *this->stream << this->str();

  this->stream->flush();

  this->str("");
  return !(*this->stream);
}
