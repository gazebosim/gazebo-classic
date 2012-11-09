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
#include <boost/lexical_cast.hpp>

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

  TiXmlDocument xmlDoc;
  if (!xmlDoc.LoadFile(_logFile))
    gzerr << "Unable to parser log file[" << _logFile << "]\n";

  this->logXml = xmlDoc.FirstChildElement("gazebo_log");
  if (!this->logXml)
    gzerr << "Log file is missing the <gazebo_log> element\n";

  //this->file.open(_logFile.c_str(), std::fstream::in);
  //if (!this->file)
  //  gzthrow("Invalid logfile[" + _logFile + "]. Unable to open for reading.\n");

  this->filename = _logFile;

  this->ReadHeader();
}

/////////////////////////////////////////////////
void LogPlay::ReadHeader()
{
  std::string logVersion, gazeboVersion;
  uint32_t randSeed;

  TiXmlElement *xml = this->logXml->FirstChildElement("header");
  if (!xml)
    gzerr << "Log file has no header\n";

  TiXmlElement *childXml = xml->FirstChildElement("log_version");
  if (!childXml)
    gzerr << "Log file header is missing the log version.\n";
  else
    logVersion = childXml->GetText();

  childXml = xml->FirstChildElement("gazebo_version");
  if (!childXml)
    gzerr << "Log file header is missing the gazebo version.\n";
  else
    gazeboVersion = childXml->GetText();

  childXml = xml->FirstChildElement("rand_seed");
  if (!childXml)
    gzerr << "Log file header is missing the random number seed.\n";
  else
    randSeed = boost::lexical_cast<uint32_t>(childXml->GetText());

  /// \TODO: add error checking for the header.
  /*this->file >> logVersion >> gazeboVersion >> randSeed;*/

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
  return false;

  if (this->file.eof())
    return false;

  std::string compression;
  uint32_t size;

  this->file >> compression >> size;
  if (compression.empty() || this->file.eof())
    return false;

  // Read the data.
  char *data = new char[size];
  this->file.read(data, size);

  _data = data;

  return !this->file.eof();
}
