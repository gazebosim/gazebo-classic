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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <gazebo/gazebo.hh>

#include <gazebo/sdf/sdf.hh>

#include <gazebo/physics/WorldState.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/gazebo_config.h>

using namespace gazebo;

std::vector<std::string> params;

/////////////////////////////////////////////////
void help()
{
  std::cerr << "This tool introspects Gazebo log files.\n"
            << "    info         : Get information about a log\n"
            << "    help         : This help text\n";
}

/////////////////////////////////////////////////
bool parse(int argc, char **argv)
{
  if (argc == 1 || std::string(argv[1]) == "help")
  {
    help();
    return false;
  }

  // Get parameters from command line
  for (int i = 1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    params.push_back(p);
  }

  // Get parameters from stdin
  if (!isatty(fileno(stdin)))
  {
    char str[1024];
    while (!feof(stdin))
    {
      if (fgets(str, 1024, stdin)== NULL)
        break;

      if (feof(stdin))
        break;
      std::string p = str;
      boost::trim(p);
      params.push_back(p);
    }
  }

  return true;
}

/////////////////////////////////////////////////
std::string get_file_size_str(const std::string &_filename)
{
  std::ostringstream size;

  // Open the file
  std::ifstream ifs(_filename.c_str());
  if (!ifs)
  {
    gzerr << "Unable to open file[" << _filename << "]\n";
    return std::string();
  }

  // Move to the end of the file
  ifs.seekg(0, std::ios::end);

  // Get the position of the file pointer, which is the number of bytes in
  // the file.
  int byteSize = ifs.tellg();

  // Generate a human friendly string
  if (byteSize < 1000)
    size << byteSize << " B";
  else if (byteSize < 1000000)
    size << byteSize / 1000.0f << " KB";
  else
    size << byteSize / 1.0e6 << " MB";

  return size.str();
}

/////////////////////////////////////////////////
void info()
{
  if (params.size() <= 1)
  {
    gzerr << "Please specify a log file\n";
    return;
  }

  gazebo::common::LogPlay *play = gazebo::common::LogPlay::Instance();

  try
  {
    play->Open(params[1]);
  }
  catch(gazebo::common::Exception &_e)
  {
    gzerr << "Unable to open log file[" << params[1] << "]\n";
    return;
  }

  // Get the SDF world description from the log file
  std::string sdfString;
  gazebo::common::LogPlay::Instance()->Step(sdfString);

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("root.sdf", sdf);
  sdf::readString(sdfString, sdf);

  gazebo::physics::WorldState state;
  boost::posix_time::ptime start, end;
  std::vector<std::string> models;

  if (sdf)
  {
    if (sdf->HasElement("model"))
    {
      sdf::ElementPtr childElem = sdf->GetElement("model");

      while (childElem)
      {
        std::string name = childElem->GetValueString("name");
        models.push_back(name);
        childElem = childElem->GetNextElement("model");
      }
    }

    // Get the log file start time.
    if (sdf->HasElement("world") &&
        sdf->GetElement("world")->HasElement("state"))
    {
      state.Load(sdf->GetElement("world")->GetElement("state"));
      start = boost::posix_time::from_time_t(state.GetWallTime().sec);
    }

    // Get the log file end time.
    std::string lastSdfString;
    if (play->GetChunk(play->GetChunkCount()-1, lastSdfString))
    {
      sdf::ElementPtr lastSdf(new sdf::Element);
      sdf::initFile("state.sdf", lastSdf);
      sdf::readString(lastSdfString, lastSdf);

      state.Load(lastSdf);
      end = boost::posix_time::from_time_t(state.GetWallTime().sec);
    }
    else
      gzerr << "Unable to read the last chunk of data in the log file.\n";
  }


  // Tell cout how to output boost dates
  boost::posix_time::time_facet *facet =
    new boost::posix_time::time_facet("%b %d %y %H:%M:%S");
  std::cout.imbue(std::locale(std::locale::classic(), facet));

  std::cout
    << "Log Version:    " << play->GetLogVersion() << "\n"
    << "Gazebo Version: " << play->GetGazeboVersion() << "\n"
    << "Random Seed:    " << play->GetRandSeed() << "\n"
    << "Start:          " << start << "\n"
    << "End:            " << end <<  "\n"
    << "Duration:       " << end - start << "\n"
    << "Steps:          " << play->GetChunkCount() << "\n"
    << "Size:           " << get_file_size_str(params[1]) << "\n"
    << "Encoding:       " << play->GetEncoding() << "\n"
    << "Model Count:    " << models.size() << "\n"
    << "\n";
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (!parse(argc, argv))
    return 0;

  if (params[0] == "info")
    info();
}
