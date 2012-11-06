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

#include <boost/filesystem.hpp>

#include "gazebo/math/Rand.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Logger.hh"
#include "gazebo/common/LogPlay.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
LogPlay::LogPlay()
{
}

/////////////////////////////////////////////////
LogPlay::~LogPlay()
{
}

/////////////////////////////////////////////////
void LogPlay::Open(const std::string &_logFile)
{
  boost::filesystem::path path(_logFile);
  if (!boost::filesystem::exists(path))
    gzthrow("Invalid logfile[" + _logFile + "]. Does not exist.\n");

  this->file.open(_logFile.c_str(), std::fstream::in);
  if (!this->file)
    gzthrow("Invalid logfile[" + _logFile + "]. Unable to open for reading.\n");

  this->filename = _logFile;
  this->ReadHeader();
}

/////////////////////////////////////////////////
void LogPlay::ReadHeader()
{
  std::string logVersion, gazeboVersion;
  uint32_t randSeed;

  /// \TODO: add error checking for the header.
  this->file >> logVersion >> gazeboVersion >> randSeed;

  std::cout << "Log Version["
            << logVersion << "] Gz["
            << gazeboVersion << "] Seed[" << randSeed << "]\n";

  if (logVersion != GZ_LOG_VERSION)
    gzwarn << "Log version[" << logVersion << "] in file[" << this->filename
           << "] does not match Gazebo's log version["
           << GZ_LOG_VERSION << "]\n";

  /// Set the random number seed for simulation
  math::Rand::SetSeed(randSeed);
}

/////////////////////////////////////////////////
bool LogPlay::IsOpen() const
{
  return this->file.is_open();
}

/////////////////////////////////////////////////
bool LogPlay::Step(std::string &_data)
{
  bool result = false;

  std::string compression;
  uint32_t size;

  this->file >> compression >> size;

  std::cout << "Compression[" << compression << "] Size[ " << size << "]\n";

  // Read the data.
  char *data = new char[size];
  this->file.read(data, size);

  _data = data;

  result = true;

  return result;
}
