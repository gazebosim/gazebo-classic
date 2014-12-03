/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/remove_whitespace.hpp>
#include <boost/archive/iterators/istream_iterator.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include "gazebo/math/Rand.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Base64.hh"
#include "gazebo/util/LogRecord.hh"
#include "gazebo/util/LogPlay.hh"

using namespace gazebo;
using namespace util;

/////////////////////////////////////////////////
LogPlay::LogPlay()
{
  this->logStartXml = NULL;
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
    gzthrow("Invalid logfile[" + _logFile + "]. Does not exist.");

  // Parse the log file
  if (!this->xmlDoc.LoadFile(_logFile))
    gzthrow("Unable to parse log file[" << _logFile << "]");

  // Get the gazebo_log element
  this->logStartXml = this->xmlDoc.FirstChildElement("gazebo_log");

  if (!this->logStartXml)
    gzthrow("Log file is missing the <gazebo_log> element");

  // Store the filename for future use.
  this->filename = _logFile;

  // Read in the header.
  this->ReadHeader();

  this->logCurrXml = this->logStartXml;
  this->encoding.clear();
}

/////////////////////////////////////////////////
std::string LogPlay::GetHeader() const
{
  std::ostringstream stream;
  stream << "<?xml version='1.0'?>\n"
         << "<gazebo_log>\n"
         << "<header>\n"
         << "<log_version>" << this->logVersion << "</log_version>\n"
         << "<gazebo_version>" << this->gazeboVersion << "</gazebo_version>\n"
         << "<rand_seed>" << this->randSeed << "</rand_seed>\n"
         << "</header>\n";

  return stream.str();
}

/////////////////////////////////////////////////
void LogPlay::ReadHeader()
{
  this->randSeed = math::Rand::GetSeed();
  TiXmlElement *headerXml, *childXml;

  this->logVersion.clear();
  this->gazeboVersion.clear();

  // Get the header element
  headerXml = this->logStartXml->FirstChildElement("header");
  if (!headerXml)
    gzthrow("Log file has no header");

  // Get the log format version
  childXml = headerXml->FirstChildElement("log_version");
  if (!childXml)
    gzerr << "Log file header is missing the log version.\n";
  else
    this->logVersion = childXml->GetText();

  // Get the gazebo version
  childXml = headerXml->FirstChildElement("gazebo_version");
  if (!childXml)
    gzerr << "Log file header is missing the gazebo version.\n";
  else
    this->gazeboVersion = childXml->GetText();

  // Get the random number seed.
  childXml = headerXml->FirstChildElement("rand_seed");
  if (!childXml)
    gzerr << "Log file header is missing the random number seed.\n";
  else
    this->randSeed = boost::lexical_cast<uint32_t>(childXml->GetText());

  if (this->logVersion != GZ_LOG_VERSION)
    gzwarn << "Log version[" << this->logVersion << "] in file["
           << this->filename
           << "] does not match Gazebo's log version["
           << GZ_LOG_VERSION << "]\n";

  /// Set the random number seed for simulation
  math::Rand::SetSeed(this->randSeed);
}

/////////////////////////////////////////////////
bool LogPlay::IsOpen() const
{
  return this->logStartXml != NULL;
}

/////////////////////////////////////////////////
std::string LogPlay::GetLogVersion() const
{
  return this->logVersion;
}

/////////////////////////////////////////////////
std::string LogPlay::GetGazeboVersion() const
{
  return this->gazeboVersion;
}

/////////////////////////////////////////////////
uint32_t LogPlay::GetRandSeed() const
{
  return this->randSeed;
}

/////////////////////////////////////////////////
bool LogPlay::Step(std::string &_data)
{
  std::string startMarker = "<sdf ";
  std::string endMarker = "</sdf>";
  size_t start = this->currentChunk.find(startMarker);
  size_t end = this->currentChunk.find(endMarker);

  if (start == std::string::npos || end == std::string::npos)
  {
    this->currentChunk.clear();

    if (this->logCurrXml == this->logStartXml)
      this->logCurrXml = this->logStartXml->FirstChildElement("chunk");
    else if (this->logCurrXml)
    {
      this->logCurrXml = this->logCurrXml->NextSiblingElement("chunk");
    }
    else
      return false;

    // Stop if there are no more chunks
    if (!this->logCurrXml)
      return false;

    if (!this->GetChunkData(this->logCurrXml, this->currentChunk))
    {
      gzerr << "Unable to decode log file\n";
      return false;
    }

    start = this->currentChunk.find(startMarker);
    end = this->currentChunk.find(endMarker);
  }

  _data = this->currentChunk.substr(start, end+endMarker.size()-start);

  this->currentChunk.erase(0, end + endMarker.size());

  return true;
}

/////////////////////////////////////////////////
bool LogPlay::GetChunk(unsigned int _index, std::string &_data)
{
  unsigned int count = 0;
  TiXmlElement *xml = this->logStartXml->FirstChildElement("chunk");

  while (xml && count < _index)
  {
    count++;
    xml = xml->NextSiblingElement("chunk");
  }

  if (xml && count == _index)
    return this->GetChunkData(xml, _data);
  else
    return false;
}

/////////////////////////////////////////////////
bool LogPlay::GetChunkData(TiXmlElement *_xml, std::string &_data)
{
  // Make sure we have valid xml pointer
  if (!_xml)
    return false;

  /// Get the chunk's encoding
  this->encoding = _xml->Attribute("encoding");

  // Make sure there is an encoding value.
  if (this->encoding.empty())
    gzthrow("Enconding missing for a chunk in log file[" +
        this->filename + "]");

  if (this->encoding == "txt")
    _data = _xml->GetText();
  else if (this->encoding == "bz2")
  {
    std::string data = _xml->GetText();
    std::string buffer;

    // Decode the base64 string
    buffer = Base64Decode(data);

    // Decompress the bz2 data
    {
      boost::iostreams::filtering_istream in;
      in.push(boost::iostreams::bzip2_decompressor());
      in.push(boost::make_iterator_range(buffer));

      // Get the data
      std::getline(in, _data, '\0');
      _data += '\0';
    }
  }
  else if (this->encoding == "zlib")
  {
    std::string data = _xml->GetText();
    std::string buffer;

    // Decode the base64 string
    buffer = Base64Decode(data);

    // Decompress the zlib data
    {
      boost::iostreams::filtering_istream in;
      in.push(boost::iostreams::zlib_decompressor());
      in.push(boost::make_iterator_range(buffer));

      // Get the data
      std::getline(in, _data, '\0');
      _data += '\0';
    }
  }
  else
  {
    gzerr << "Inavlid encoding[" << this->encoding << "] in log file["
      << this->filename << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
std::string LogPlay::GetEncoding() const
{
  return this->encoding;
}

/////////////////////////////////////////////////
unsigned int LogPlay::GetChunkCount() const
{
  unsigned int count = 0;
  TiXmlElement *xml = this->logStartXml->FirstChildElement("chunk");

  while (xml)
  {
    count++;
    xml = xml->NextSiblingElement("chunk");
  }

  return count;
}
