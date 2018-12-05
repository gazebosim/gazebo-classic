/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <tinyxml.h>

#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/fuel_tools/FuelClient.hh>
#include <sdf/sdf.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/FuelModelDatabase.hh"
#include "gazebo/common/SemanticVersion.hh"
#include "gazebo/common/SystemPaths.hh"

using namespace gazebo;
using namespace common;

FuelModelDatabase *FuelModelDatabase::myself = FuelModelDatabase::Instance();

/// \brief Private class attributes for FuelModelDatabase.
class gazebo::common::FuelModelDatabasePrivate
{
  /// \brief A client to interact with Ignition Fuel.
  public: std::unique_ptr<ignition::fuel_tools::FuelClient> fuelClient;
};

/////////////////////////////////////////////////
FuelModelDatabase::FuelModelDatabase()
  : dataPtr(new FuelModelDatabasePrivate)
{
  unsigned int verbosity = common::Console::GetQuiet() ? 0 : 4;
  ignition::common::Console::SetVerbosity(verbosity);

  ignition::fuel_tools::ClientConfig conf;
  conf.LoadConfig();
  std::string userAgent = "Gazebo " GAZEBO_VERSION_FULL;

#if defined(_WIN32)
  userAgent += "(Windows)";
#elif defined(__linux__)
  userAgent += "(GNU/Linux)";
#elif defined(__APPLE__)
  userAgent += "(Mac OSX)";
#elif defined(BSD)
  userAgent += "(BSD)";
#else
  userAgent += "(unknown)";
#endif

  conf.SetUserAgent(userAgent);
  this->dataPtr->fuelClient.reset(new ignition::fuel_tools::FuelClient(conf));

  common::SystemPaths::Instance()->AddFindFileCallback(std::bind(
      &FuelModelDatabase::CachedFilePath, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
FuelModelDatabase::~FuelModelDatabase()
{
}

/////////////////////////////////////////////////
std::vector<ignition::fuel_tools::ServerConfig> FuelModelDatabase::Servers()
  const
{
  return this->dataPtr->fuelClient->Config().Servers();
}

/////////////////////////////////////////////////
void FuelModelDatabase::Models(
    const ignition::fuel_tools::ServerConfig &_server, std::function <void
    (const std::vector<ignition::fuel_tools::ModelIdentifier> &)> &_func)
{
  std::thread t([this, _func, _server]
  {
    // Run the callback passing the list of models.
    _func(this->Models(_server));
  });
  t.detach();
}

/////////////////////////////////////////////////
std::vector<ignition::fuel_tools::ModelIdentifier> FuelModelDatabase::Models(
    const ignition::fuel_tools::ServerConfig &_server) const
{
  std::vector<ignition::fuel_tools::ModelIdentifier> models;

  for (auto iter = this->dataPtr->fuelClient->Models(_server); iter; ++iter)
  {
    models.push_back(iter->Identification());
  }

  return models;
}

/////////////////////////////////////////////////
std::string FuelModelDatabase::ModelFile(const std::string &_uri)
{
  std::string result;

  // This will download the model if necessary
  std::string path = this->ModelPath(_uri);
  std::string manifestPath =
    ignition::common::joinPaths(path, GZ_MODEL_MANIFEST_FILENAME);

  // Get the GZ_MODEL_MANIFEST_FILENAME.
  if (!ignition::common::exists(manifestPath))
  {
    gzerr << "Missing " << GZ_MODEL_MANIFEST_FILENAME
          << " for model " << path << "\n";
    return result;
  }

  TiXmlDocument xmlDoc;
  SemanticVersion sdfParserVersion(SDF_VERSION);
  std::string bestVersionStr = "0.0";
  if (xmlDoc.LoadFile(manifestPath))
  {
    TiXmlElement *modelXML = xmlDoc.FirstChildElement("model");
    if (modelXML)
    {
      TiXmlElement *sdfXML = modelXML->FirstChildElement("sdf");
      TiXmlElement *sdfSearch = sdfXML;

      // Find the SDF element that matches our current SDF version.
      // If a match is not found, use the latest version of the element
      // that is not older than the SDF parser.
      while (sdfSearch)
      {
        if (sdfSearch->Attribute("version"))
        {
          std::string version = std::string(sdfSearch->Attribute("version"));
          SemanticVersion modelVersion(version);
          SemanticVersion bestVersion(bestVersionStr);
          if (modelVersion > bestVersion)
          {
            // this model is better than the previous one
            if (modelVersion <= sdfParserVersion)
            {
              // the parser can read it
              sdfXML = sdfSearch;
              bestVersionStr = version;
            }
            else
            {
              gzwarn << "Ignoring version " << version
                << " for model " << _uri
                << " because Gazebo is using an older sdf parser (version "
                << SDF_VERSION << ")" << std::endl;
            }
          }
        }
        sdfSearch = sdfSearch->NextSiblingElement("sdf");
      }

      if (sdfXML)
      {
        result = path + "/" + sdfXML->GetText();
      }
      else
      {
        gzerr << "Manifest[" << manifestPath << "] doesn't have "
              << "<model><sdf>...</sdf></model> element.\n";
      }
    }
    else
    {
      gzerr << "Manifest[" << manifestPath
            << "] doesn't have a <model> element\n";
    }
  }
  else
  {
    gzerr << "Invalid model manifest file[" << manifestPath << "]\n";
  }

  return result;
}

/////////////////////////////////////////////////
std::string FuelModelDatabase::ModelPath(const std::string &_uri,
    const bool _forceDownload)
{
  auto fuelUri = ignition::common::URI(_uri);

  if (fuelUri.Scheme() != "http" && fuelUri.Scheme() != "https")
  {
    gzwarn << "URI not supported by Fuel [" << _uri << "]" << std::endl;
    return std::string();
  }

  std::string path;

  if (!_forceDownload)
  {
    if (this->dataPtr->fuelClient->CachedModel(fuelUri, path))
    {
      return path;
    }
  }

  if (!this->dataPtr->fuelClient->DownloadModel(fuelUri.Str(), path))
  {
    gzerr << "Unable to download model[" << _uri << "]" << std::endl;
    return std::string();
  }
  else
  {
      gzmsg << "Downloaded model URL: " << std::endl
            << "          " << _uri << std::endl
            << "      to: " << std::endl
            << "          " << path << std::endl;
  }

  return path;
}

/////////////////////////////////////////////////
std::string FuelModelDatabase::CachedFilePath(const std::string &_uri)
{
  auto fuelUri = ignition::common::URI(_uri);
  if (fuelUri.Scheme() != "http" && fuelUri.Scheme() != "https")
  {
    gzwarn << "URI not supported by Fuel [" << _uri << "]" << std::endl;
    return std::string();
  }

  std::string path;
  if (!this->dataPtr->fuelClient->CachedModelFile(fuelUri, path))
  {
    gzerr << "Unable to download model[" << _uri << "]" << std::endl;
  }

  return path;
}

