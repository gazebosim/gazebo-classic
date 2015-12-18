/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <algorithm>
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

#include <ignition/math/Rand.hh>

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Base64.hh"
#include "gazebo/util/LogRecord.hh"

#include "gazebo/util/LogPlayPrivate.hh"
#include "gazebo/util/LogPlay.hh"

using namespace gazebo;
using namespace util;

/////////////////////////////////////////////////
LogPlay::LogPlay()
: dataPtr(new LogPlayPrivate)
{
  this->dataPtr->logStartXml = NULL;
}

/////////////////////////////////////////////////
LogPlay::~LogPlay()
{
}

/////////////////////////////////////////////////
void LogPlay::Open(const std::string &_logFile)
{
  this->dataPtr->currentChunk.clear();

  boost::filesystem::path path(_logFile);
  if (!boost::filesystem::exists(path))
    gzthrow("Invalid logfile [" + _logFile + "]. Does not exist.");

  if (boost::filesystem::is_directory(path))
    gzthrow("Invalid logfile [" + _logFile + "]. This is a directory.");

  // Parse the log file
  if (this->dataPtr->xmlDoc.LoadFile(_logFile.c_str()) !=
      tinyxml2::XML_NO_ERROR)
  {
    gzthrow("Unable to parse log file[" << _logFile << "]");
  }

  // Get the gazebo_log element
  this->dataPtr->logStartXml =
    this->dataPtr->xmlDoc.FirstChildElement("gazebo_log");

  if (!this->dataPtr->logStartXml)
    gzthrow("Log file is missing the <gazebo_log> element");

  // Store the filename for future use.
  this->dataPtr->filename = _logFile;

  // Read in the header.
  this->ReadHeader();

  this->dataPtr->logCurrXml = this->dataPtr->logStartXml;
  this->dataPtr->encoding.clear();

  // Extract the start/end log times from the log.
  this->ReadLogTimes();

  // Extract the initial "iterations" value from the log.
  this->dataPtr->iterationsFound = this->ReadIterations();

  this->dataPtr->logCurrXml =
    this->dataPtr->logStartXml->FirstChildElement("chunk");

  if (!this->dataPtr->logCurrXml)
    gzthrow("Unable to find the first chunk");

  if (!this->ChunkData(this->dataPtr->logCurrXml, this->dataPtr->currentChunk))
    gzthrow("Unable to decode log file");

  this->dataPtr->start = 0;
  this->dataPtr->end = -1 * this->dataPtr->kEndFrame.size();
}

/////////////////////////////////////////////////
std::string LogPlay::GetHeader() const
{
  return this->Header();
}

/////////////////////////////////////////////////
std::string LogPlay::Header() const
{
  std::ostringstream stream;
  stream << "<?xml version='1.0'?>\n"
         << "<gazebo_log>\n"
         << "<header>\n"
         << "<log_version>" << this->dataPtr->logVersion << "</log_version>\n"
         << "<gazebo_version>" << this->dataPtr->gazeboVersion
         << "</gazebo_version>\n"
         << "<rand_seed>" << this->dataPtr->randSeed << "</rand_seed>\n"
         << "<log_start>" << this->dataPtr->logStartTime << "</log_start>\n"
         << "<log_end>" << this->dataPtr->logEndTime << "</log_end>\n"
         << "</header>\n";

  return stream.str();
}

/////////////////////////////////////////////////
uint64_t LogPlay::GetInitialIterations() const
{
  return this->InitialIterations();
}

/////////////////////////////////////////////////
uint64_t LogPlay::InitialIterations() const
{
  return this->dataPtr->initialIterations;
}

/////////////////////////////////////////////////
bool LogPlay::HasIterations() const
{
  return this->dataPtr->iterationsFound;
}

/////////////////////////////////////////////////
void LogPlay::ReadHeader()
{
  this->dataPtr->randSeed = ignition::math::Rand::Seed();
  tinyxml2::XMLElement *headerXml, *childXml;

  this->dataPtr->logVersion.clear();
  this->dataPtr->gazeboVersion.clear();

  // Get the header element
  headerXml = this->dataPtr->logStartXml->FirstChildElement("header");
  if (!headerXml)
    gzthrow("Log file has no header");

  // Get the log format version
  childXml = headerXml->FirstChildElement("log_version");
  if (!childXml)
    gzerr << "Log file header is missing the log version.\n";
  else
    this->dataPtr->logVersion = childXml->GetText();

  if (this->dataPtr->logVersion != GZ_LOG_VERSION)
  {
    gzwarn << "Log version[" << this->dataPtr->logVersion << "] in file["
           << this->dataPtr->filename
           << "] does not match Gazebo's log version["
           << GZ_LOG_VERSION << "]\n";
    return;
  }

  // Get the gazebo version
  childXml = headerXml->FirstChildElement("gazebo_version");
  if (!childXml)
    gzerr << "Log file header is missing the gazebo version.\n";
  else
    this->dataPtr->gazeboVersion = childXml->GetText();

  // Get the random number seed.
  childXml = headerXml->FirstChildElement("rand_seed");
  if (!childXml)
  {
    gzerr << "Log file header is missing the random number seed.\n";
  }
  else
  {
    this->dataPtr->randSeed = std::stoul(childXml->GetText());
  }

  // Set the random number seed for simulation
  ignition::math::Rand::Seed(this->dataPtr->randSeed);
}

/////////////////////////////////////////////////
void LogPlay::ReadLogTimes()
{
  std::string chunk;
  bool found = false;

  auto chunkXml = this->dataPtr->logStartXml->FirstChildElement("chunk");

  // Try to read the start time of the log.
  auto numChunksToTry =
    std::min(this->ChunkCount(), this->dataPtr->kNumChunksToTry);

  for (unsigned int i = 0; i < numChunksToTry; ++i)
  {
    if (!chunkXml)
    {
      gzerr << "Unable to find the first chunk" << std::endl;
      return;
    }

    if (!this->ChunkData(chunkXml, chunk))
      return;

    // Find the first <sim_time> of the log.
    auto from = chunk.find(this->dataPtr->kStartTime);
    auto to = chunk.find(this->dataPtr->kEndTime,
        from + this->dataPtr->kStartTime.size());
    if (from != std::string::npos && to != std::string::npos)
    {
      auto length = to - from - this->dataPtr->kStartTime.size();
      auto startTime = chunk.substr(from + this->dataPtr->kStartTime.size(),
          length);
      std::stringstream ss(startTime);
      ss >> this->dataPtr->logStartTime;
      found = true;
      break;
    }

    chunkXml = chunkXml->NextSiblingElement("chunk");
  }

  if (!found)
    gzwarn << "Unable to find <sim_time> tags in any chunk." << std::endl;

  // Jump to the last chunk for finding the last <sim_time>.
  auto lastChunk = this->dataPtr->logStartXml->LastChildElement("chunk");
  if (!lastChunk)
  {
    gzerr << "Unable to jump to the last chunk of the log file\n";
    return;
  }

  if (!this->ChunkData(lastChunk, chunk))
    return;

  // Update the last <sim_time> of the log.
  auto to = chunk.rfind(this->dataPtr->kEndTime);
  auto from = chunk.rfind(this->dataPtr->kStartTime, to - 1);

  if (from != std::string::npos && to != std::string::npos)
  {
    auto length = to - from - this->dataPtr->kStartTime.size();
    auto endTime = chunk.substr(
        from + this->dataPtr->kStartTime.size(), length);
    std::stringstream ss(endTime);
    ss >> this->dataPtr->logEndTime;
  }
  else
  {
    gzwarn << "Unable to find <sim_time>...</sim_time> tags in the last chunk."
           << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
bool LogPlay::ReadIterations()
{
  const std::string kStartDelim = "<iterations>";
  const std::string kEndDelim = "</iterations>";

  auto chunkXml = this->dataPtr->logStartXml->FirstChildElement("chunk");

  // Read the first "iterations" value of the log from the first chunk.
  auto numChunksToTry =
    std::min(this->ChunkCount(), this->dataPtr->kNumChunksToTry);

  for (unsigned int i = 0; i < numChunksToTry; ++i)
  {
    if (!chunkXml)
    {
      gzerr << "Unable to find the first chunk" << std::endl;
      return false;
    }

    std::string chunk;
    if (!this->ChunkData(chunkXml, chunk))
      return false;

    // Find the first <iterations> of the log.
    auto from = chunk.find(kStartDelim);
    auto to = chunk.find(kEndDelim, from + kStartDelim.size());
    if (from != std::string::npos && to != std::string::npos)
    {
      auto length = to - from - kStartDelim.size();
      auto iterations = chunk.substr(from + kStartDelim.size(), length);
      std::stringstream ss(iterations);
      ss >> this->dataPtr->initialIterations;
      return true;
    }

    chunkXml = chunkXml->NextSiblingElement("chunk");
  }

  gzwarn << "Unable to find <iterations>...</iterations> tags in the first "
         << "chunk. Assuming that the first <iterations> value is 0."
         << std::endl;
  return false;
}

/////////////////////////////////////////////////
bool LogPlay::IsOpen() const
{
  return this->dataPtr->logStartXml != NULL;
}

/////////////////////////////////////////////////
std::string LogPlay::GetLogVersion() const
{
  return this->LogVersion();
}

/////////////////////////////////////////////////
std::string LogPlay::LogVersion() const
{
  return this->dataPtr->logVersion;
}

/////////////////////////////////////////////////
std::string LogPlay::GetGazeboVersion() const
{
  return this->GazeboVersion();
}

/////////////////////////////////////////////////
std::string LogPlay::GazeboVersion() const
{
  return this->dataPtr->gazeboVersion;
}

/////////////////////////////////////////////////
uint32_t LogPlay::GetRandSeed() const
{
  return this->RandSeed();
}

/////////////////////////////////////////////////
uint32_t LogPlay::RandSeed() const
{
  return this->dataPtr->randSeed;
}

/////////////////////////////////////////////////
common::Time LogPlay::GetLogStartTime() const
{
  return this->LogStartTime();
}

/////////////////////////////////////////////////
common::Time LogPlay::LogStartTime() const
{
  return this->dataPtr->logStartTime;
}

/////////////////////////////////////////////////
common::Time LogPlay::GetLogEndTime() const
{
  return this->LogEndTime();
}

/////////////////////////////////////////////////
common::Time LogPlay::LogEndTime() const
{
  return this->dataPtr->logEndTime;
}

/////////////////////////////////////////////////
std::string LogPlay::GetFilename() const
{
  return this->Filename();
}

/////////////////////////////////////////////////
std::string LogPlay::Filename() const
{
  return boost::filesystem::basename(this->dataPtr->filename) +
    boost::filesystem::extension(this->dataPtr->filename);
}

/////////////////////////////////////////////////
std::string LogPlay::GetFullPathFilename() const
{
  return this->FullPathFilename();
}

/////////////////////////////////////////////////
std::string LogPlay::FullPathFilename() const
{
  const boost::filesystem::path logFilename(this->dataPtr->filename);
  return boost::filesystem::canonical(logFilename).string();
}

/////////////////////////////////////////////////
uintmax_t LogPlay::GetFileSize() const
{
  return this->FileSize();
}

/////////////////////////////////////////////////
uintmax_t LogPlay::FileSize() const
{
  return boost::filesystem::file_size(this->dataPtr->filename);
}

/////////////////////////////////////////////////
bool LogPlay::Step(std::string &_data)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  auto from = this->dataPtr->currentChunk.find(this->dataPtr->kStartFrame,
      this->dataPtr->end + this->dataPtr->kEndFrame.size());
  auto to = this->dataPtr->currentChunk.find(this->dataPtr->kEndFrame,
      this->dataPtr->end + this->dataPtr->kEndFrame.size());

  if (from == std::string::npos || to == std::string::npos)
  {
    if (!this->NextChunk())
      return false;

    from = this->dataPtr->currentChunk.find(this->dataPtr->kStartFrame);
    to = this->dataPtr->currentChunk.find(this->dataPtr->kEndFrame);
    if (from == std::string::npos || to == std::string::npos)
    {
      gzerr << "Unable to find an <sdf> frame in current chunk\n";
      return false;
    }
  }

  this->dataPtr->start = from;
  this->dataPtr->end = to;

  _data = this->dataPtr->currentChunk.substr(this->dataPtr->start,
      this->dataPtr->end + this->dataPtr->kEndFrame.size() -
      this->dataPtr->start);

  return true;
}

/////////////////////////////////////////////////
bool LogPlay::Step(const int _step, std::string &_data)
{
  bool res = false;
  for (auto i = 0; i < std::abs(_step); ++i)
  {
    if (_step >= 0)
    {
      if (!this->Step(_data))
        return res;
    }
    else
    {
      if (!this->StepBack(_data))
        return res;
    }

    // If at least one of the steps was successfuly executed we'll return true.
    res = true;
  }

  return res;
}

/////////////////////////////////////////////////
bool LogPlay::StepBack(std::string &_data)
{
  auto from = std::string::npos;
  auto to = std::string::npos;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->start > 0)
  {
    from = this->dataPtr->currentChunk.rfind(
        this->dataPtr->kStartFrame, this->dataPtr->start - 1);
    to = this->dataPtr->currentChunk.rfind(
        this->dataPtr->kEndFrame, this->dataPtr->start - 1);
  }

  if (this->dataPtr->start <= 0 || from == std::string::npos ||
      to == std::string::npos)
  {
    if (!this->PrevChunk())
      return false;

    from = this->dataPtr->currentChunk.rfind(this->dataPtr->kStartFrame);
    to = this->dataPtr->currentChunk.rfind(this->dataPtr->kEndFrame);
    if (from == std::string::npos || to == std::string::npos)
    {
      gzerr << "Unable to find an <sdf> frame in current chunk\n";
      return false;
    }
  }

  this->dataPtr->start = from;
  this->dataPtr->end = to;

  _data = this->dataPtr->currentChunk.substr(this->dataPtr->start,
      this->dataPtr->end + this->dataPtr->kEndFrame.size() -
      this->dataPtr->start);

  return true;
}

/////////////////////////////////////////////////
bool LogPlay::Rewind()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->currentChunk.clear();
  this->dataPtr->logCurrXml =
    this->dataPtr->logStartXml->FirstChildElement("chunk");

  if (!this->dataPtr->logCurrXml)
  {
    gzerr << "Unable to jump to the beginning of the log file\n";
    return false;
  }

  if (!this->ChunkData(this->dataPtr->logCurrXml, this->dataPtr->currentChunk))
    return false;

  // Skip first <sdf> block (it doesn't have a world state).
  this->dataPtr->end = this->dataPtr->currentChunk.find(
      this->dataPtr->kEndFrame);
  if (this->dataPtr->end == std::string::npos)
  {
    std::cerr << "Unable to find the first <sdf> block" << std::endl;
    return false;
  }

  // Remove the special first <sdf> block.
  this->dataPtr->currentChunk.erase(
      0, this->dataPtr->end + this->dataPtr->kEndFrame.size());

  this->dataPtr->start = 0;
  this->dataPtr->end = -1 * this->dataPtr->kEndFrame.size();

  return true;
}

/////////////////////////////////////////////////
bool LogPlay::Forward()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Get the last chunk.
  this->dataPtr->logCurrXml =
    this->dataPtr->logStartXml->LastChildElement("chunk");

  if (!this->dataPtr->logCurrXml)
  {
    gzerr << "Unable to jump to the end of the log file\n";
    return false;
  }

  if (!this->ChunkData(this->dataPtr->logCurrXml, this->dataPtr->currentChunk))
    return false;

  this->dataPtr->start = this->dataPtr->currentChunk.size() - 1;
  this->dataPtr->end = this->dataPtr->currentChunk.size() - 1;

  return true;
}

/////////////////////////////////////////////////
bool LogPlay::Seek(const common::Time &_time)
{
  if (_time >= this->dataPtr->logEndTime)
  {
    this->Forward();
    std::string frame;
    this->Step(-2, frame);
    return true;
  }

  common::Time logTime = this->dataPtr->logStartTime;

  // 1st step: Locate the chunk: We're looking for the first chunk that has
  // a time greater than the target time.
  int64_t imin = 0;
  int64_t imax = this->ChunkCount() - 1;
  while (imin <= imax)
  {
    int64_t imid = imin + ((imax - imin) / 2);
    this->Chunk(imid, this->dataPtr->currentChunk);

    this->dataPtr->start = 0;
    this->dataPtr->end = -1 * this->dataPtr->kEndFrame.size();

    // We try a few times looking for <sim_time>.
    for (unsigned int i = 0; i < 2; ++i)
    {
      std::string frame;
      if (!this->Step(frame))
        return false;

      // Search the <sim_time> in the first frame of the current chunk.
      auto from = frame.find(this->dataPtr->kStartTime);
      auto to = frame.find(
          this->dataPtr->kEndTime, from + this->dataPtr->kStartTime.size());
      if (from != std::string::npos && to != std::string::npos)
      {
        auto length = to - from - this->dataPtr->kStartTime.size();
        auto logTimeStr = frame.substr(
            from + this->dataPtr->kStartTime.size(), length);
        std::stringstream ss(logTimeStr);
        ss >> logTime;
        break;
      }
    }

    // Chunk found.
    if (logTime == _time)
      break;
    else if (logTime < _time)
      imin = imid + 1;
    else
      imax = imid - 1;
  }

  if (logTime < _time)
  {
    if (!this->NextChunk())
      this->Forward();
  }

  // 2nd step: Locate the frame in the previous chunk.
  while (true)
  {
    std::string frame;
    if (!this->StepBack(frame))
      break;

    // Search the <sim_time> in the frame of the current chunk.
    auto from = frame.find(this->dataPtr->kStartTime);
    auto to = frame.find(
        this->dataPtr->kEndTime, from + this->dataPtr->kStartTime.size());
    if (from != std::string::npos && to != std::string::npos)
    {
      auto length = to - from - this->dataPtr->kStartTime.size();
      auto logTimeStr = frame.substr(
          from + this->dataPtr->kStartTime.size(), length);
      std::stringstream ss(logTimeStr);
      ss >> logTime;

      // frame found.
      if (logTime < _time)
        break;
    }
  }

  return true;
}

/////////////////////////////////////////////////
bool LogPlay::GetChunk(unsigned int _index, std::string &_data)
{
  return this->Chunk(_index, _data);
}

/////////////////////////////////////////////////
bool LogPlay::Chunk(unsigned int _index, std::string &_data) const
{
  unsigned int count = 0;
  this->dataPtr->logCurrXml =
    this->dataPtr->logStartXml->FirstChildElement("chunk");

  while (this->dataPtr->logCurrXml && count < _index)
  {
    count++;
    this->dataPtr->logCurrXml =
      this->dataPtr->logCurrXml->NextSiblingElement("chunk");
  }

  if (this->dataPtr->logCurrXml && count == _index)
    return this->ChunkData(this->dataPtr->logCurrXml, _data);
  else
    return false;
}

/////////////////////////////////////////////////
bool LogPlay::ChunkData(tinyxml2::XMLElement *_xml, std::string &_data) const
{
  // Make sure we have valid xml pointer
  if (!_xml)
  {
    gzerr << "NULL XML element" << std::endl;
    return false;
  }

  /// Get the chunk's encoding
  this->dataPtr->encoding = _xml->Attribute("encoding");

  // Make sure there is an encoding value.
  if (this->dataPtr->encoding.empty())
  {
    gzthrow("Encoding missing for a chunk in log file[" +
        this->dataPtr->filename + "]");
  }

  if (this->dataPtr->encoding == "txt")
    _data = _xml->GetText();
  else if (this->dataPtr->encoding == "bz2")
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
  else if (this->dataPtr->encoding == "zlib")
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
    gzerr << "Invalid encoding[" << this->dataPtr->encoding << "] in log file["
      << this->dataPtr->filename << "]\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
std::string LogPlay::GetEncoding() const
{
  return this->Encoding();
}

/////////////////////////////////////////////////
std::string LogPlay::Encoding() const
{
  return this->dataPtr->encoding;
}

/////////////////////////////////////////////////
unsigned int LogPlay::GetChunkCount() const
{
  return this->ChunkCount();
}

/////////////////////////////////////////////////
unsigned int LogPlay::ChunkCount() const
{
  unsigned int count = 0;
  auto xml = this->dataPtr->logStartXml->FirstChildElement("chunk");

  while (xml)
  {
    count++;
    xml = xml->NextSiblingElement("chunk");
  }

  return count;
}

/////////////////////////////////////////////////
bool LogPlay::NextChunk()
{
  auto next = this->dataPtr->logCurrXml->NextSiblingElement("chunk");
  if (!next)
    return false;

  this->dataPtr->logCurrXml = next;
  if (!this->ChunkData(this->dataPtr->logCurrXml, this->dataPtr->currentChunk))
    return false;

  this->dataPtr->start = 0;
  this->dataPtr->end = -1 * this->dataPtr->kEndFrame.size();

  return true;
}

/////////////////////////////////////////////////
bool LogPlay::PrevChunk()
{
  auto prev = this->dataPtr->logCurrXml->PreviousSiblingElement("chunk");
  if (!prev)
    return false;

  this->dataPtr->logCurrXml = prev;
  if (!this->ChunkData(this->dataPtr->logCurrXml, this->dataPtr->currentChunk))
    return false;

  this->dataPtr->start = this->dataPtr->currentChunk.size() - 1;
  this->dataPtr->end = this->dataPtr->currentChunk.size() - 1;

  return true;
}
