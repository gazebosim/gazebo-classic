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
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/filesystem.hpp>
#include <ignition/fuel-tools.hh>
#include <sdf/sdf.hh>

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
  // ToDo: Remove this block when Ignition Fuel Tools supports parsing
  // a configuration file.
  ignition::fuel_tools::ServerConfig srv;
  srv.URL("https://staging-api.ignitionfuel.org");
  ignition::fuel_tools::ClientConfig conf;
  conf.AddServer(srv);

  this->dataPtr->fuelClient.reset(new ignition::fuel_tools::FuelClient(conf));
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
    const ignition::fuel_tools::ServerConfig &_server,
    std::function<void (const std::map<std::string, std::string> &)> &_func)
{
  std::thread t([this, _func, _server]
  {
    // Run the callback passing the list of models.
    _func(this->Models(_server));
  });
  t.detach();
}

/////////////////////////////////////////////////
std::map<std::string, std::string> FuelModelDatabase::Models(
    const ignition::fuel_tools::ServerConfig &_server) const
{
  std::map<std::string, std::string> models;

  for (auto iter = this->dataPtr->fuelClient->Models(_server); iter; ++iter)
  {
    std::string fullURI = iter->Identification().UniqueName();
    std::string modelName = iter->Identification().Name();

    models[fullURI] = modelName;
  }

  return models;
}

/////////////////////////////////////////////////
std::string FuelModelDatabase::ModelFile(const std::string &_uri)
{
  std::string result;

  // This will download the model if necessary
  std::string path = this->ModelPath(_uri);

  boost::filesystem::path manifestPath = path;

  // Get the GZ_MODEL_MANIFEST_FILENAME.
  if (boost::filesystem::exists(manifestPath / GZ_MODEL_MANIFEST_FILENAME))
    manifestPath /= GZ_MODEL_MANIFEST_FILENAME;
  else
  {
    gzerr << "Missing " << GZ_MODEL_MANIFEST_FILENAME
      << " for model " << manifestPath << "\n";
  }

  TiXmlDocument xmlDoc;
  SemanticVersion sdfParserVersion(SDF_VERSION);
  std::string bestVersionStr = "0.0";
  if (xmlDoc.LoadFile(manifestPath.string()))
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
    bool _forceDownload)
{
  std::string path;

  if (!_forceDownload)
    path = SystemPaths::Instance()->FindFileURI(_uri);

  struct stat st;
  if (path.empty() || stat(path.c_str(), &st) != 0 )
  {
    // Try to download the model.
    if (!this->dataPtr->fuelClient->DownloadModel(_uri, path))
    {
      gzerr << "Unable to download model[" << _uri << "]" << std::endl;
      return std::string();
    }
  }

  return path;
}
