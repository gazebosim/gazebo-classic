/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <curl/curl.h>
#include <boost/filesystem.hpp>

#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/SphericalCoordinates.hh>
#include <ignition/math/Vector2.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/transport/Node.hh>

#include "StaticMapPlugin.hh"

namespace gazebo
{
  /// \brief Class to provide helper functions for Web Mercator projection
  class MercatorProjection
  {
    /// \brief Convert point in world coordinates to latitude and longitude
    /// \param[in] _point Point in world coordinates
    /// \return Latitude and longitude coorindates
    public: static ignition::math::SphericalCoordinates PointToLatLon(
        const ignition::math::Vector2d &_point);

    /// \brief Convert latitdue and longitude to point in world coordinates
    /// \param[in] Latitude and longitude coorindates
    /// \return Point in world coordinates
    public: static ignition::math::Vector2d LatLonToPoint(
        const ignition::math::SphericalCoordinates &_latLon);

    /// \brief Google map base level tile size
    public: static const unsigned int TILE_SIZE;
  };


  /// \brief Private data class for StaticMapPlugin
  class StaticMapPluginPrivate
  {
    /// \brief Download map tiles.
    /// \param[in] _centerLat Latitude of center point of map
    /// \param[in] _centerLon Longitude of center point of map
    /// \param[in] _zoom Map zoom level between 0 (entire world) and 21+
    /// (streets).
    /// \param[in] _tileSizePx Size of each map tile in pixels. Tiles will be
    /// square.
    /// \param[in] _worldSize Size of map in the world in meters.
    /// \param[in] _mapType Type of map to download: roadmap, satellite,
    /// terrain, hybrid
    /// \param[in] _apiKey Google API key
    /// \param[in] _saveDirPath Location in local filesystem to save tile
    /// images.
    public: std::vector<std::string> DownloadMapTiles(const double _centerLat,
        const double _centerLon, const unsigned int _zoom,
        const unsigned int _tileSizePx,
        const ignition::math::Vector2d &_worldSize,
        const std::string &_mapType, const std::string &_apiKey,
        const std::string &_saveDirPath);

    /// \brief Create textured map model and save it in specified path.
    /// \param[in] _name Name of map model
    /// \param[in] _tileWorldSize Size of map tiles in meters
    /// \param[in] _xNumTiles Number of tiles in x direction
    /// \param[in] _yNumTiles Number of tiles in y direction
    /// \param[in] _tiles Tile image filenames
    /// \param[in] _modelPath Path to model directory
    /// \return True if map tile model has been successfully created.
    public: bool CreateMapTileModel(
        const std::string &_name,
        const double _tileWorldSize,
        const unsigned int xNumTiles, const unsigned int yNumTiles,
        const std::vector<std::string> &_tiles, const std::string &_modelPath);

    /// \brief Get the ground resolution at the specified latitude and zoom
    /// level.
    /// \param[in] _lat Latitude
    /// \param[in] _zoom Map zoom Level
    /// \return Ground resolution in meters per pixel.
    public: double GroundResolution(const double _lat,
        const unsigned int _zoom) const;

    /// \brief Spawn a model into the world
    /// \param[in] _name Name of model
    /// \param[in] _pose Pose of model
    public: void SpawnModel(const std::string &_name,
        const ignition::math::Pose3d &_pose);

    /// \brief Pointer to world.
    public: physics::WorldPtr world;

    /// \brief Name of map model
    public: std::string modelName;

    /// \brief Pose of map model
    public: ignition::math::Pose3d modelPose;

    /// \brief Latitude and Longitude of map center
    public: ignition::math::Vector2d center;

    /// \brief Target size of world to be covered by map in meters.
    public: ignition::math::Vector2d worldSize;

    /// \brief Map zoom level. From 0 (entire world) to 21+ (streets)
    public: unsigned int zoom = 21u;

    /// \brief Size of map tile in pixels. 640 is max resolution for users of
    /// standard API
    public: unsigned int tileSizePx = 640u;

    /// \brief Type of map to use as texture: roadmap, satellite, terrain,
    /// hybrid
    public: std::string mapType = "satellite";

    /// \brief True to use cached model and image data from gazebo model path.
    /// False to redownload image tiles and recreate model sdf and config
    /// files.
    public: bool useCache = false;

    /// \brief Google API key
    public: std::string apiKey;

    /// \brief Filenames of map tile images
    public: std::vector<std::string> mapTileFilenames;

    /// \brief Pointer to a node for communication.
    public: transport::NodePtr node;

    /// \brief Factory publisher.
    public: transport::PublisherPtr factoryPub;

    /// \brief True if the plugin is loaded successfully
    public: bool loaded = false;
  };
}

using namespace gazebo;

const unsigned int MercatorProjection::TILE_SIZE = 256;

GZ_REGISTER_WORLD_PLUGIN(StaticMapPlugin)


/////////////////////////////////////////////////
size_t WriteData(void *_ptr, size_t _size, size_t _nmemb, FILE *_stream)
{
  return fwrite(_ptr, _size, _nmemb, _stream);
}

/////////////////////////////////////////////////
bool DownloadFile(const std::string &_url, const std::string &_outputFile)
{
  if (_url.empty())
    return false;

  CURL *curl = curl_easy_init();

  curl_easy_setopt(curl, CURLOPT_URL, _url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteData);

  FILE *fp = fopen(_outputFile.c_str(), "wb");
  if (!fp)
  {
    gzerr << "Could not download model[" << _url << "] because we were"
      << "unable to write to file[" << _outputFile << "]."
      << "Please fix file permissions.";
    return false;
  }
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);

  char errbuf[CURL_ERROR_SIZE];
  // provide a buffer to store errors in
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errbuf);
  // set the error buffer as empty before performing a request
  errbuf[0] = 0;

  CURLcode success = curl_easy_perform(curl);
  if (success != CURLE_OK)
  {
    gzerr << "Error in REST request" << std::endl;
    size_t len = strlen(errbuf);
    fprintf(stderr, "\nlibcurl: (%d) ", success);
    if (len)
    {
      fprintf(stderr, "%s%s", errbuf,
              ((errbuf[len - 1] != '\n') ? "\n" : ""));
    }
    else
      fprintf(stderr, "%s\n", curl_easy_strerror(success));
  }
  fclose(fp);

  // Update the status code.
  int statusCode = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &statusCode);

  // Cleaning.
  curl_easy_cleanup(curl);

  return true;
}


/////////////////////////////////////////////////
ignition::math::Vector2d MercatorProjection::LatLonToPoint(
    const ignition::math::SphericalCoordinates &_latLon)
{
  // Adapted from:
  // https://developers.google.com/maps/documentation/javascript/examples/map-coordinates

  ignition::math::Vector2d point;
  // Truncating to 0.9999 effectively limits latitude to 89.189. This is
  // about a third of a tile past the edge of the world tile.
  double siny = std::min(std::max(std::sin(
      _latLon.LatitudeReference().Radian()), -0.9999), 0.9999);

  point.X() = TILE_SIZE * (0.5 + _latLon.LongitudeReference().Degree()/ 360.0);
  point.Y() = TILE_SIZE * (0.5 - std::log((1 + siny) / (1 - siny)) /
          (4 * IGN_PI));
  return point;
}

/////////////////////////////////////////////////
ignition::math::SphericalCoordinates MercatorProjection::PointToLatLon(
    const ignition::math::Vector2d &_point)
{
  ignition::math::SphericalCoordinates latLon;
  ignition::math::Angle lonAngle;
  ignition::math::Angle latAngle;

  double lonDegrees = (_point.X() / TILE_SIZE - 0.5) * 360;
  double latRadians = (_point.Y() - TILE_SIZE/2.0) /
        -(TILE_SIZE/(2*IGN_PI));

  lonAngle.Degree(lonDegrees);
  latAngle.Radian(2 * std::atan(std::exp(latRadians)) - IGN_PI / 2.0);

  latLon.SetLongitudeReference(lonAngle);
  latLon.SetLatitudeReference(latAngle);
  return latLon;
}

/////////////////////////////////////////////////
StaticMapPlugin::StaticMapPlugin()
  : dataPtr(new StaticMapPluginPrivate)
{
}

/////////////////////////////////////////////////
void StaticMapPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->dataPtr->world = _world;

  if (!_sdf->HasElement("api_key"))
  {
    gzerr << "Missing Google API key needed to download map tiles" << std::endl;
    return;
  }

  if (!_sdf->HasElement("center"))
  {
    gzerr << "Please specify latitude and longitude coordinates of map center"
          << std::endl;
    return;
  }

  if (!_sdf->HasElement("world_size"))
  {
    gzerr << "Please specify size of map to cover in meters" << std::endl;
    return;
  }

  this->dataPtr->apiKey = _sdf->Get<std::string>("api_key");
  this->dataPtr->center =
      _sdf->Get<ignition::math::Vector2d>("center");
  double wSize = _sdf->Get<double>("world_size");
  if (wSize > 0)
  {
    // support only square map for now
    this->dataPtr->worldSize.X() = wSize;
    this->dataPtr->worldSize.Y() = wSize;
  }
  else
  {
    gzerr << "World size must be greater than 0 meters" << std::endl;
    return;
  }

  // optional params
  if (_sdf->HasElement("zoom"))
    this->dataPtr->zoom = _sdf->Get<unsigned int>("zoom");

  if (_sdf->HasElement("tile_size"))
  {
    this->dataPtr->tileSizePx = _sdf->Get<unsigned int>("tile_size");
    if (this->dataPtr->tileSizePx > 640u)
    {
      gzerr << "Tile size exceeds standard API usage limit. Setting to 640px."
            << std::endl;
      this->dataPtr->tileSizePx = 640u;
    }
  }

  if (_sdf->HasElement("map_type"))
    this->dataPtr->mapType= _sdf->Get<std::string>("map_type");

  if (_sdf->HasElement("use_cache"))
    this->dataPtr->useCache = _sdf->Get<bool>("use_cache");

  if (_sdf->HasElement("pose"))
    this->dataPtr->modelPose = _sdf->Get<ignition::math::Pose3d>("pose");

  if (_sdf->HasElement("model_name"))
    this->dataPtr->modelName = _sdf->Get<std::string>("model_name");
  else
  {
    // generate name based on input
    std::stringstream name;
    name << "map_" << this->dataPtr->mapType << "_" << std::setprecision(9)
         << this->dataPtr->center.X() << "_" << this->dataPtr->center.Y()
         << "_" << this->dataPtr->worldSize.X()
         << "_" << this->dataPtr->worldSize.Y();
    this->dataPtr->modelName = name.str();
  }

  this->dataPtr->loaded = true;
}

/////////////////////////////////////////////////
void StaticMapPlugin::Init()
{
  // don't init if params are not loaded successfully
  if (!this->dataPtr->loaded)
    return;

  // check if model exists locally
  auto basePath = common::SystemPaths::Instance()->GetLogPath() /
        boost::filesystem::path("models");

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->factoryPub =
      this->dataPtr->node->Advertise<msgs::Factory>("~/factory");

  boost::filesystem::path modelPath = basePath / this->dataPtr->modelName;
  if (this->dataPtr->useCache && common::exists(modelPath.string()))
  {
    gzmsg << "Model: '" << this->dataPtr->modelName << "' exists. "
          << "Spawning existing model.." << std::endl;
    this->dataPtr->SpawnModel("model://" + this->dataPtr->modelName,
        this->dataPtr->modelPose);
    return;
  }


  // create tmp dir to save model files
  boost::filesystem::path tmpModelPath =
      boost::filesystem::temp_directory_path() / this->dataPtr->modelName;
  boost::filesystem::path scriptsPath(tmpModelPath / "materials" / "scripts");
  boost::filesystem::create_directories(scriptsPath);
  boost::filesystem::path texturesPath(tmpModelPath / "materials" / "textures");
  boost::filesystem::create_directories(texturesPath);

  // download map tile images into model/materials/textures
  std::vector<std::string> tiles = this->dataPtr->DownloadMapTiles(
      this->dataPtr->center.X(),
      this->dataPtr->center.Y(),
      this->dataPtr->zoom,
      this->dataPtr->tileSizePx,
      this->dataPtr->worldSize,
      this->dataPtr->mapType,
      this->dataPtr->apiKey,
      texturesPath.string());

  // assume square model for now
  unsigned int xNumTiles = std::sqrt(tiles.size());
  unsigned int yNumTiles = xNumTiles;

  double tileWorldSize = this->dataPtr->GroundResolution(
      IGN_DTOR(this->dataPtr->center.X()), this->dataPtr->zoom)
      * this->dataPtr->tileSizePx;

  // create model and spawn it into the world
  if (this->dataPtr->CreateMapTileModel(
      this->dataPtr->modelName, tileWorldSize,
      xNumTiles, yNumTiles, tiles, tmpModelPath.string()))
  {
    // verify model dir is created
    if (common::exists(tmpModelPath.string()))
    {
      // remove existing map model
      if (common::exists(modelPath.string()))
        boost::filesystem::remove_all(modelPath);

      try
      {
        // move new map model to gazebo model path
        boost::filesystem::rename(tmpModelPath, modelPath);
      }
      catch(boost::filesystem::filesystem_error &_e)
      {
        // rename failed. Could be an invalid cross-device link error
        // try copy and remove method
        bool result = common::copyDir(tmpModelPath, modelPath);
        if (result)
        {
          boost::filesystem::remove_all(tmpModelPath);
        }
        else
        {
          gzerr<< "Unable to copy model from '" << tmpModelPath.string()
                 << "' to '" << modelPath.string() << "'" << std::endl;
          return;
        }
      }
      // spawn the model
      this->dataPtr->SpawnModel("model://" + this->dataPtr->modelName,
          this->dataPtr->modelPose);
    }
    else
      gzerr << "Failed to create model: " << tmpModelPath.string() << std::endl;
  }
}

/////////////////////////////////////////////////
double StaticMapPluginPrivate::GroundResolution(const double _lat,
    const unsigned int _zoom) const
{
  double earthEquatorialRadius = 6378137;
  double metersPerPx = 2 * IGN_PI * earthEquatorialRadius *
      std::cos(_lat) / (MercatorProjection::TILE_SIZE * std::pow(2, _zoom));
  return metersPerPx;
}

/////////////////////////////////////////////////
std::vector<std::string> StaticMapPluginPrivate::DownloadMapTiles(
    const double _centerLat, const double _centerLon,
    const unsigned int _zoom, const unsigned int _tileSizePx,
    const ignition::math::Vector2d &_worldSize, const std::string &_mapType,
    const std::string &_apiKey, const std::string &_saveDirPath)
{
  ignition::math::Angle lonAngle;
  ignition::math::Angle latAngle;
  latAngle.Degree(_centerLat);
  lonAngle.Degree(_centerLon);
  ignition::math::SphericalCoordinates centerLatLon;
  centerLatLon.SetLatitudeReference(latAngle);
  centerLatLon.SetLongitudeReference(lonAngle);

  // ground resolution - varies by latitude and zoom level
  double metersPerPx =
      this->GroundResolution(centerLatLon.LatitudeReference().Radian(), _zoom);

  // determine number of tiles necessary to cover specified world size
  unsigned int xNumTiles = static_cast<unsigned int>(
      std::ceil(_worldSize.X() / metersPerPx /
      _tileSizePx));
  // y is only approximate because ground resolution is based on latitude
  unsigned int yNumTiles = static_cast<unsigned int>(
      std::ceil(_worldSize.Y() / metersPerPx /
      _tileSizePx));

  // scale for converting between pixel and world point
  double scale = std::pow(2, _zoom);

  ignition::math::Vector2d centerPx =
      MercatorProjection::LatLonToPoint(centerLatLon) * scale;

  // compute starting x, y values in pixel coordinates
  double halfWidthPx = _tileSizePx * std::floor(xNumTiles / 2);
  double halfHeightPx = _tileSizePx * std::floor(yNumTiles / 2);
  double halfTileSize = _tileSizePx / 2.0;
  double x = centerPx.X() - halfWidthPx;
  double y = centerPx.Y() - halfHeightPx;
  if (xNumTiles % 2 == 0u)
    x += halfTileSize;
  if (yNumTiles % 2 == 0u)
    y += halfTileSize;
  double startx = x;

  // download map tiles using google static map API
  std::string url = "https://maps.googleapis.com/maps/api/staticmap";
  for (unsigned int i = 0; i < yNumTiles; ++i)
  {
    for (unsigned int j = 0; j < xNumTiles; ++j)
    {
      // convert from pixels to world point
      auto px = ignition::math::Vector2d(x, y);
      auto point = px / scale;
      // convert world point to lat lon
      auto latLon = MercatorProjection::PointToLatLon(point);

      // download tile image
      std::stringstream query;
      query << "?center="
            << std::setprecision(9)
            << latLon.LatitudeReference().Degree() << ","
            << latLon.LongitudeReference().Degree()
            << "&zoom=" << _zoom
            << "&size=" << _tileSizePx << "x" << _tileSizePx
            << "&maptype=" << _mapType
            << "&key=" << _apiKey;
      std::string fullURL = url + query.str();
      std::stringstream filename;
      filename << "tile_"
               << std::setprecision(9) << latLon.LatitudeReference().Degree()
               << "_" << latLon.LongitudeReference().Degree() << ".png";
      std::string fullPath = _saveDirPath + "/" + filename.str();
      DownloadFile(fullURL, fullPath);
      gzmsg << "Downloading map tile: " << filename.str() << std::endl;
      mapTileFilenames.push_back(filename.str());

      x += _tileSizePx;
    }
    x = startx;
    y += _tileSizePx;
  }
  return mapTileFilenames;
}

/////////////////////////////////////////////////
bool StaticMapPluginPrivate::CreateMapTileModel(
    const std::string &_name,
    const double _tileWorldSize,
    const unsigned int _xNumTiles, const unsigned int _yNumTiles,
    const std::vector<std::string> &_tiles, const std::string &_modelPath)
{
  // create material script
  std::stringstream materialScriptStr;
  for (unsigned int i = 0; i < _yNumTiles; ++i)
  {
    for (unsigned int j = 0; j < _xNumTiles; ++j)
    {
      materialScriptStr <<
        "material " << _name << "/" << i << "_" << j << "\n"
        "{\n"
        "  technique\n"
        "  {\n"
        "    pass\n"
        "    {\n"
        "      texture_unit\n"
        "      {\n"
        "        texture " << _tiles[j + i * _xNumTiles] << "\n"
        "      }\n"
        "    }\n"
        "  }\n"
        "}\n\n";
    }
  }

  // save material script file to disk
  boost::filesystem::path scriptFilePath(_modelPath);
  scriptFilePath = scriptFilePath / "materials" / "scripts"
      / "map_tiles.material";
  std::ofstream scriptFile;
  scriptFile.open(scriptFilePath.string().c_str());
  if (!scriptFile.is_open())
  {
    gzerr << "Couldn't open file for writing: " << scriptFilePath.string()
          << std::endl;
    return false;
  }
  scriptFile << materialScriptStr.str();
  scriptFile.close();

  // create model.sdf file
  double sizeX = _tileWorldSize;
  double sizeY = _tileWorldSize;
  double sizeZ = 1.0;

  ignition::math::Vector2d wCenter(0, 0);
  double halfTileWorldWidth = sizeX * std::floor(_xNumTiles / 2);
  double halfTileWorldHeight = sizeY * std::floor(_yNumTiles / 2);
  double x = wCenter.X() - halfTileWorldWidth;
  double y = wCenter.Y() + halfTileWorldHeight;
  double halfTileWorldSizeX = sizeX / 2.0;
  double halfTileWorldSizeY = sizeY / 2.0;
  if (_xNumTiles % 2 == 0u)
    x += halfTileWorldSizeX;
  if (_yNumTiles % 2 == 0u)
    y -= halfTileWorldSizeY;
  double startx = x;
  double zPos = -sizeZ / 2.0;

  // rotate around z to line up textures
  ignition::math::Vector3d tileRot(0, 0, IGN_PI / 2.0);
  ignition::math::Vector3d colSize(sizeX * _xNumTiles, sizeY * _yNumTiles,
      sizeZ);

  // Model will have boxed-shaped tiles with z size of 1.0
  // Surface of model will be at z=0.0
  std::stringstream newModelStr;
  newModelStr << "<sdf version='" << SDF_VERSION << "'>\n"
    "<model name='" << _name << "'>\n"
    "  <static>true</static>\n"
    "  <link name='link'>\n"
    "    <collision name='collision'>\n"
    "      <pose>0 0 " << zPos << " 0 0 0</pose>\n"
    "      <geometry>\n"
    "        <box>\n"
    "          <size>" << colSize << "</size>\n"
    "        </box>\n"
    "      </geometry>\n"
    "    </collision>\n";
  for (unsigned int i = 0; i < _yNumTiles; ++i)
  {
    for (unsigned int j = 0; j < _xNumTiles; ++j)
    {
      newModelStr <<
        "    <visual name='visual" << i << "_" << j <<"'>\n"
        "      <pose>" << x << " " << y << " " << zPos
                       << " " << tileRot << "</pose>\n"
        "      <geometry>\n"
        "        <box>\n"
        "          <size>" << sizeX << " " << sizeY << " " << sizeZ
                           << "</size>\n"
        "        </box>\n"
        "      </geometry>\n"
        "      <material>\n"
        "        <script>\n"
        "          <uri>model://" << _name << "/materials/scripts</uri>\n"
        "          <uri>model://" << _name << "/materials/textures</uri>\n"
        "          <name>" << _name << "/" << i << "_" << j << "</name>\n"
        "        </script>\n"
        "      </material>\n"
        "    </visual>\n";
      x += sizeX;
    }
    x = startx;
    y -= sizeY;
  }
  newModelStr <<
    "  </link>\n"
    "</model>\n"
    "</sdf>";

  // save model.sdf file to disk
  boost::filesystem::path modelSDFFilePath(_modelPath);
  modelSDFFilePath /= "model.sdf";
  std::ofstream modelSDFFile;
  modelSDFFile.open(modelSDFFilePath.string().c_str());
  if (!modelSDFFile.is_open())
  {
    gzerr << "Couldn't open file for writing: " << modelSDFFilePath.string()
          << std::endl;
    return false;
  }
  modelSDFFile << newModelStr.str();
  modelSDFFile.close();

  // create model.config file
  std::ostringstream modelConfigStr;
  modelConfigStr << "<?xml version=\"1.0\"?>\n"
  << "<model>\n"
  << "  <name>" << _name << "</name>\n"
  << "  <version>1.0</version>\n"
  << "  <sdf version=\"" << SDF_VERSION << "\">model.sdf</sdf>\n"
  << "  <author>\n"
  << "    <name>gazebo</name>\n"
  << "    <email></email>\n"
  << "  </author>\n"
  << "  <description>\n"
  << "    Made with Gazebo using Google Static Map API. "
  <<     "https://developers.google.com/maps/documentation/static-maps\n"
  << "  </description>\n"
  << "</model>";

  // save model.config file to disk
  boost::filesystem::path modelConfigFilePath(_modelPath);
  modelConfigFilePath /= "model.config";
  std::ofstream modelConfigFile;
  modelConfigFile.open(modelConfigFilePath.string().c_str());
  if (!modelConfigFile.is_open())
  {
    gzerr << "Couldn't open file for writing: "
        << modelConfigFilePath.string() << std::endl;
    return false;
  }
  modelConfigFile << modelConfigStr.str();
  modelConfigFile.close();

  return true;
}

/////////////////////////////////////////////////
void StaticMapPluginPrivate::SpawnModel(const std::string &_uri,
    const ignition::math::Pose3d &_pose)
{
  // publish to factory topic to spawn the model
  msgs::Factory msg;
  msg.set_sdf_filename(_uri);
  msgs::Set(msg.mutable_pose(), _pose);
  this->factoryPub->Publish(msg);
}
