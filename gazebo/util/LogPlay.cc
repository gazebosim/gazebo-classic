/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/remove_whitespace.hpp>
#include <boost/archive/iterators/istream_iterator.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include "gazebo/math/Rand.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/util/LogRecord.hh"
#include "gazebo/util/LogPlay.hh"

using namespace gazebo;
using namespace util;

/////////////////////////////////////////////////
// Convert a Base64 string.
// We have to use our own function, instead of just using the
// boost::archive iterators, because the boost::archive iterators throw an
// error when the end of the Base64 string is reached. The expection then
// causes nothing to happen.
// TLDR; Boost is broken.
void base64_decode(std::string &_dest, const std::string &_src)
{
  typedef boost::archive::iterators::transform_width<
    boost::archive::iterators::binary_from_base64<const char*>, 8, 6>
    base64_dec;

  try
  {
    base64_dec srcIter(_src.c_str());
    for (unsigned int i = 0; i < _src.size(); ++i)
    {
      _dest += *srcIter;
      ++srcIter;
    }
  }
  catch(boost::archive::iterators::dataflow_exception &)
  {
  }
}

/////////////////////////////////////////////////
LogPlay::LogPlay()
{
  this->needsStep = false;
  this->pause = false;
  this->chunkCount = 0;
  this->segmentCount = 0;
  this->chunkCount = 0;
  this->logStartXml = NULL;
  this->xmlDoc = NULL;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("/gazebo");

  this->logControlSub = this->node->Subscribe("/gazebo/log/play/control",
      &LogPlay::OnLogControl, this);
  this->logStatusPub = this->node->Advertise<msgs::LogPlayStatus>(
      "/gazebo/log/play/status", 10000);
}

/////////////////////////////////////////////////
LogPlay::~LogPlay()
{
  delete this->xmlDoc;
  this->xmlDoc = NULL;
}

/////////////////////////////////////////////////
void LogPlay::Open(const std::string &_logFile)
{
  boost::filesystem::path path(_logFile);
  if (!boost::filesystem::exists(path))
    gzthrow("Invalid logfile[" + _logFile + "]. Does not exist.");

  if (this->xmlDoc)
    delete this->xmlDoc;
  this->xmlDoc = new TiXmlDocument();

  // Parse the log file
  if (!this->xmlDoc->LoadFile(_logFile))
    gzthrow("Unable to parse log file[" << _logFile << "]");

  // Get the gazebo_log element
  this->logStartXml = this->xmlDoc->FirstChildElement("gazebo_log");

  if (!this->logStartXml)
    gzthrow("Log file is missing the <gazebo_log> element");

  // Store the filename for future use.
  this->filename = _logFile;

  // Read in the header.
  this->ReadHeader();

  this->logCurrXml = this->logStartXml;
  this->encoding.clear();

  this->currentChunk.clear();

  this->CalculateStepCount();
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
  this->needsStep = false;

  if (this->currentStep < this->stepBuffer.size())
    _data = this->stepBuffer[this->currentStep];

  if (this->pause)
    return true;

  if (this->currentStep >= this->stepBuffer.size())
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

    this->stepBuffer.push_back(_data);

    this->currentChunk.erase(0, end + endMarker.size());
  }

  ++this->currentStep;

  this->PublishStatus();

  return true;
}

/////////////////////////////////////////////////
uint64_t LogPlay::GetSegmentCount() const
{
  return this->segmentCount;
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
    base64_decode(buffer, data);

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
  return this->chunkCount;
}

/////////////////////////////////////////////////
void LogPlay::CalculateStepCount()
{
  std::string data;

  this->segmentCount = 0;
  this->chunkCount = 0;

  TiXmlElement *xml = this->logStartXml->FirstChildElement("chunk");

  printf("Processing Log Chunk     ");
  fflush(stdout);

  while (xml)
  {
     printf("\b\b\b\b%04lu", this->chunkCount);
     fflush(stdout);

    if (this->logVersion == "1.0")
    {
      this->GetChunkData(xml, data);
      std::string tag = "<sdf ";
      size_t pos = 0;
      while ((pos = data.find(tag, pos)) != std::string::npos)
      {
        ++this->segmentCount;
        pos += tag.size();
      }
    }
    else
    {
      data = xml->Attribute("segments");
      try
      {
        this->segmentCount += boost::lexical_cast<uint64_t>(data);
      }
      catch(...)
      {
        gzerr << "Invalid segment count in log file. Unable to evalute["
          << data << "]\n";
      }
    }

    xml = xml->NextSiblingElement("chunk");
    ++this->chunkCount;
  }

  printf("\n");
}

/////////////////////////////////////////////////
bool LogPlay::NeedsStep()
{
  return this->needsStep;
}

/////////////////////////////////////////////////
void LogPlay::OnLogControl(ConstLogPlayControlPtr &_data)
{
  if (_data->has_target_step())
  {
    std::cout << "Targe step[" << _data->target_step() << "]\n";
    this->currentStep = _data->target_step();
    this->needsStep = true;
  }

  if (_data->has_pause())
  {
    this->pause = _data->pause();
  }
}

/////////////////////////////////////////////////
void LogPlay::PublishStatus()
{
  msgs::LogPlayStatus msg;
  msg.set_chunks(this->chunkCount);
  msg.set_segments(this->segmentCount);
  msg.set_step(this->currentStep);

  this->logStatusPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlay::Fini()
{
  if (this->node)
    this->node->Fini();
  this->node.reset();
}
