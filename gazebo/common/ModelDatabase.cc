/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#define BOOST_FILESYSTEM_VERSION 2

#include <tinyxml.h>
#include <curl/curl.h>
#include <boost/filesystem.hpp>

#include "common/SystemPaths.hh"
#include "common/Console.hh"
#include "common/ModelDatabase.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelPath(const std::string &_uri)
{
  std::string path = SystemPaths::Instance()->FindFileURI(_uri);

  if (path.empty())
  {
    gzerr << "Path is not local[" << _uri << "]. Must download\n";
  }

  return path;
}

/////////////////////////////////////////////////
std::string ModelDatabase::GetModelFile(const std::string &_uri)
{
  std::string result;

  // This will download the model if necessary
  std::string path = ModelDatabase::GetModelPath(_uri);

  std::string manifest = path + "/manifest.xml";

  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(manifest))
  {
    TiXmlElement *modelXML = xmlDoc.FirstChildElement("model");
    if (modelXML)
    {
      TiXmlElement *sdfXML = modelXML->FirstChildElement("sdf");
      if (sdfXML)
      {
        result = path + "/" + sdfXML->GetText();
      }
      else
      {
        gzerr << "Manifest[" << manifest << "] doesn't have "
              << "<model><sdf>...</sdf></model> element.\n";
      }
    }
    else
    {
      gzerr << "Manifest[" << manifest << "] doesn't have a <model> element\n";
    }
  }
  else
  {
    gzerr << "Invalid model manifest file[" << manifest << "]\n";
  }

  return result;
}
