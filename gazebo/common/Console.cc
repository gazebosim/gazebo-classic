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
#include <string.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Console.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
Console::Console()
{
  this->msgStream = &std::cerr;
  this->errStream = &std::cerr;
  this->logStream = NULL;
  this->quiet = false;
}

//////////////////////////////////////////////////
Console::~Console()
{
  if (this->logStream)
    this->logStream->close();
}

//////////////////////////////////////////////////
void Console::Init(const std::string &_logFilename)
{
  if (!getenv("HOME"))
    gzthrow("Missing HOME environment variable");

  boost::filesystem::path logPath(getenv("HOME"));
  logPath = logPath / ".gazebo/" / _logFilename;

  // If the logPath is a directory, just rename it.
  if (boost::filesystem::is_directory(logPath))
  {
    std::string newPath = logPath.string() + ".old";
    boost::system::error_code ec;
    boost::filesystem::rename(logPath, newPath, ec);
    if (ec == 0)
      std::cerr << "Deprecated log directory [" << logPath
                << "] renamed to [" << newPath << "]" << std::endl;
    else
    {
      std::cerr << "Unable to rename deprecated log directory [" << logPath
                << "] to [" << newPath << "]. Reason: " << ec.message();
      return;
    }
  }

  if (this->logStream)
  {
    this->logStream->close();
    delete this->logStream;
  }

  this->logStream = new std::ofstream(logPath.string().c_str(), std::ios::out);

  if (!this->logStream->is_open())
    std::cerr << "Error opening log file: " << logPath << std::endl;
}

//////////////////////////////////////////////////
bool Console::IsInitialized() const
{
  return this->logStream != NULL;
}

//////////////////////////////////////////////////
void Console::SetQuiet(bool _quiet)
{
  this->quiet = _quiet;
}

//////////////////////////////////////////////////
bool Console::GetQuiet() const
{
  return this->quiet;
}

//////////////////////////////////////////////////
std::ostream &Console::ColorMsg(const std::string &_lbl, int _color)
{
  if (this->quiet)
    return this->nullStream;
  else
  {
    *this->msgStream << "\033[1;" << _color << "m" << _lbl << "\033[0m ";
    return *this->msgStream;
  }
}

//////////////////////////////////////////////////
std::ofstream &Console::Log()
{
  if (!this->logStream)
    this->logStream = new std::ofstream("/dev/null", std::ios::out);

  *this->logStream << "[" << common::Time::GetWallTime() << "] ";
  this->logStream->flush();
  return *this->logStream;
}

//////////////////////////////////////////////////
std::ostream &Console::ColorErr(const std::string &lbl,
                                const std::string &file,
                                unsigned int line, int color)
{
  int index = file.find_last_of("/") + 1;

  *this->errStream << "\033[1;" << color << "m" << lbl << " [" <<
    file.substr(index , file.size() - index)<< ":" << line << "]\033[0m ";

  return *this->errStream;
}
