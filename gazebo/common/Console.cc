/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include <string.h>
#include <boost/filesystem.hpp>
#include <sstream>

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/gazebo_config.h"

using namespace gazebo;
using namespace common;

FileLogger gazebo::common::Console::g_log("");
Logger Console::g_msg("{Msg}", 32, &std::cout);
Logger Console::g_err("{Err}", 31, &std::cerr);
Logger Console::g_dbg("{Dbg}", 36, &std::cout);
Logger Console::g_warn("{Wrn}", 33, &std::cerr);

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
Logger::Logger(const std::string &_prefix, int _color, std::ostream *_stream)
  : std::ostream(new Buffer(_prefix, _color, _stream))
{
}

/////////////////////////////////////////////////
Logger::~Logger()
{
  delete this->rdbuf();
}

/////////////////////////////////////////////////
Logger &Logger::operator()(const std::string &_file, int _line)
{
  int index = _file.find_last_of("/") + 1;
  (*this) << "[" << _file.substr(index , _file.size() - index) << ":"
    << _line << "]";

  return (*this);
}

/////////////////////////////////////////////////
Logger::Buffer::Buffer(const std::string &_prefix, int _color,
    std::ostream *_stream)
  : color(_color), stream(_stream), prefix(_prefix)
{
  this->log = &Console::g_log;
}

/////////////////////////////////////////////////
Logger::Buffer::~Buffer()
{
  this->log = NULL;
  this->pubsync();
}

/////////////////////////////////////////////////
int Logger::Buffer::sync()
{
  // Always log
  if (this->log)
  {
    (*this->log) << this->prefix << " " << this->str() << std::endl;
  }

  // Output to terminal
  if (!Console::GetQuiet())
  {
    (*this->stream) << "\033[1;" 
      << this->color << "m" << this->str() << "\033[0m";
  }

  this->str("");
  return !(*this->stream);
}

/////////////////////////////////////////////////
FileLogger::FileLogger(const std::string &_filename)
  : std::ostream(new Buffer(_filename))
{
}

/////////////////////////////////////////////////
FileLogger::~FileLogger()
{
  delete this->rdbuf();
}

/////////////////////////////////////////////////
void FileLogger::Init(const std::string &_filename)
{
  FileLogger::Buffer *buf = static_cast<FileLogger::Buffer*>(
      this->rdbuf());

  // Only allow initialization once.
  if (buf->stream)
    return;

  if (!getenv("HOME"))
    gzthrow("Missing HOME environment variable");

  boost::filesystem::path logPath(getenv("HOME"));
  logPath = logPath / ".gazebo/" / _filename;

  buf->stream = new std::ofstream(logPath.string().c_str(), std::ios::out);

  // Output the version of Gazebo.
  (*buf->stream) << GAZEBO_VERSION_HEADER << std::endl;
}

/////////////////////////////////////////////////
FileLogger::Buffer::Buffer(const std::string &_filename)
  : stream(NULL)
{
  if (!_filename.empty())
  {
    this->stream = new std::ofstream(_filename.c_str(), std::ios::out);
  }
}

/////////////////////////////////////////////////
FileLogger::Buffer::~Buffer()
{
  if (this->stream)
    static_cast<std::ofstream*>(this->stream)->close();
}

/////////////////////////////////////////////////
int FileLogger::Buffer::sync()
{
  if (!this->stream)
    return -1;

  *this->stream << "(" << common::Time::GetWallTime() << ") " << this->str();

  this->stream->flush();

  this->str("");
  return !(*this->stream);
}
