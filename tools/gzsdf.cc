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

#include <sys/stat.h>

#include <sdf/sdf.hh>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

std::vector<std::string> params;

using namespace sdf;

/////////////////////////////////////////////////
void help()
{
  std::cerr << "gzsdf -- Tool to provide information about SDF files.\n\n";
  std::cerr << "`gzsdf` <command>\n\n";
  std::cerr << "This tool provides information about SDF files.\n\n";
  std::cerr << "Commands:\n";
  std::cerr << "    describe <SDF version>     Print the SDF format.\n";
  std::cerr << "    convert <file>             "
    << "In place conversion to the latest format.\n";
  std::cerr << "    doc <SDF version>          Print HTML SDF.\n";
  std::cerr << "    check <file> <SDF version> Check the SDF format for the";
  std::cerr << " given file.\n";
  std::cerr << "    print <SDF version>         Prints SDF, useful for ";
  std::cerr << " debugging and as a conversion tool.\n\n";

  std::cerr << "See also:\n"
    << "Example and more information about gazebo gzsdf and other command"
    << "line tools can be found at: "
    << "http://gazebosim.org/user_guide/started__commandlinetools.html\n\n"
    << "For more information about the SDF format please read: "
    << "http://gazebosim.org/sdf.html\n";
}

/////////////////////////////////////////////////
bool file_exists(const std::string &_filename)
{
  struct stat st;
  return stat(_filename.c_str(), &st) == 0;
}

/////////////////////////////////////////////////
int main(int argc, char** argv)
{
  bool success = false;

  try
  {
    // Initialize the informational logger. This will log warnings and errors.
    gazebo::common::Console::Instance()->Init("gzsdf.log");
  }
  catch(gazebo::common::Exception &_e)
  {
    _e.Print();
    std::cerr << "Error initializing log file" << std::endl;
  }

  // Get parameters from command line
  for (int i = 1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    params.push_back(p);
  }

  if (params.empty() || params[0] == "help" || params[0] == "-h")
  {
    help();
    return 0;
  }

  // We must set the findFile callback here so that gzsdf check/print
  // can find resource files when parsing the sdf in readFile().
  sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

  if ((params[0] == "check" || params[0] == "print" || params[0] == "convert"))
  {
    if (params.size() == 3)
      SDF::version = params[2];
  }
  else if (params.size() == 2)
    SDF::version = params[1];

  boost::shared_ptr<SDF> sdf(new SDF());
  if (!init(sdf))
  {
    std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
    return -1;
  }

  if (params[0] == "check")
  {
    if (params.size() < 2)
    {
      help();
      std::cerr << "Error: Expecting an xml file to parse\n\n";
      return -1;
    }

    if (!file_exists(params[1]))
      std::cerr << "Error: File doesn't exist[" << params[1] << "]\n";

    if (!readFile(params[1], sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      return -1;
    }

    success = true;
    std::cout << "Check complete\n";
  }
  else if (params[0] == "describe")
  {
    sdf->PrintDescription();
    success = true;
  }
  else if (params[0] == "doc")
  {
    sdf->PrintDoc();
    success = true;
  }
  else if (params[0] == "convert")
  {
    if (params.size() < 2)
    {
      help();
      std::cerr << "Error: Missing SDF file to convert\n\n";
      return -1;
    }

    if (!file_exists(params[1]))
      std::cerr << "Error: File doesn't exist[" << params[1] << "]\n";

    TiXmlDocument xmlDoc;
    if (xmlDoc.LoadFile(params[1]))
    {
      if (sdf::Converter::Convert(&xmlDoc, SDF::version, true))
      {
        success = true;

        // Create an XML printer to control formatting
        TiXmlPrinter printer;
        printer.SetIndent("  ");
        xmlDoc.Accept(&printer);

        // Output the XML
        std::ofstream stream(params[1].c_str(), std::ios_base::out);
        stream << printer.Str();
        stream.close();
      }
    }
    else
      std::cerr << "Unable to load file[" << params[1] << "]\n";
  }
  else if (params[0] == "print")
  {
    if (params.size() < 2)
    {
      help();
      std::cerr << "Error: Expecting an xml file to parse\n\n";
      return -1;
    }

    if (!file_exists(params[1]))
      std::cerr << "Error: File doesn't exist[" << params[1] << "]\n";

    if (!readFile(params[1], sdf))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      return -1;
    }
    success = true;
    sdf->PrintValues();
  }
  else
  {
    help();
    std::cerr << "Error: Unknown option[" << params[0] << "]\n";
  }

  if (params[0] != "print" && params[0] != "doc" && success)
    std::cout << "Success\n";
  return 0;
}
