/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>
#include <vector>

#include "gazebo/common/Exception.hh"
#include "gazebo/util/ZadeatGTParser.hh"

using namespace gazebo;
using namespace util;

//////////////////////////////////////////////////
ZadeatGTParser::ZadeatGTParser(const std::string &_filename)
{
  // Sanity check.
  boost::filesystem::path srcPath(_filename);
  if (!boost::filesystem::exists(srcPath) ||
      !boost::filesystem::is_regular_file(srcPath))
    gzthrow("Invalid GT log file [" + _filename + "]. Does not exist.");

  this->logFile.open(_filename.c_str());

}

//////////////////////////////////////////////////
ZadeatGTParser::~ZadeatGTParser()
{
  this->logFile.close();
}

//////////////////////////////////////////////////
std::istream & ZadeatGTParser::GetNextGT(double &_timestamp,
    double &_headRoll, double &_headPitch, double &_headYaw,
    double &_headX, double &_headY, double &_headZ,
    double &_torsoRoll, double &_torsoPitch, double &_torsoYaw,
    double &_torsoX, double &_torsoY, double &_torsoZ)
{
  std::string line;
  std::vector<std::string> splitVec;

  if (this->logFile.is_open())
  { 
    getline(this->logFile, line);
    //isComment = boost::starts_with(line, "#");

    // Parse the line looking for the fields
    boost::erase_all(line, " ");
    boost::split(splitVec, line, boost::is_any_of(","));

    std::vector<std::string>::iterator it = splitVec.begin();
    _timestamp = boost::lexical_cast<double>(*it++);
    _headRoll = boost::lexical_cast<double>(*it++);
    _headPitch = boost::lexical_cast<double>(*it++);
    _headYaw = boost::lexical_cast<double>(*it++);
    _headX = boost::lexical_cast<double>(*it++);
    _headY = boost::lexical_cast<double>(*it++);
    _headZ = boost::lexical_cast<double>(*it++);
    _torsoRoll = boost::lexical_cast<double>(*it++);
    _torsoPitch = boost::lexical_cast<double>(*it++);
    _torsoYaw = boost::lexical_cast<double>(*it++);
    _torsoX = boost::lexical_cast<double>(*it++);
    _torsoY = boost::lexical_cast<double>(*it++);
    _torsoZ = boost::lexical_cast<double>(*it++);
  }

  return this->logFile;
}