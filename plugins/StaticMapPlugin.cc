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

#include <curl/curl.h>
#include <boost/filesystem.hpp>

#include <ignition/math/Vector2.hh>
#include <ignition/math/SphericalCoordinates.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/CommonIface.hh>
#include <gazebo/transport/Node.hh>

#include "StaticMapPlugin.hh"

namespace gazebo
{
  class StaticMapPluginPrivate
  {
    /// \brief Download map tiles.
    public: std::vector<std::string> DownloadMapTiles(const double _centerLat, 
        const double _centerLon, const double _zoom, const unsigned int _tileSizePx,
        const ignition::math::Vector2d &_worldSize, const std::string &_apiKey,
        const std::string &_saveLocation);

    /// \brief Create the textured map object and insert into world.
    public: void CreateMapTileModel(
        const std::string &_name, 
        const double _tileWorldSize,
        const unsigned int xNumTiles, const unsigned int yNumTiles,
        const std::vector<std::string> &_tiles, const std::string &_modelPath);

    public: double GroundResolution(const double _lat, 
        const unsigned int _zoom);

    /// \brief Spawn a model into the world
    public: void SpawnModel(const std::string &_name);

    /// \brief Pointer to world.
    public: physics::WorldPtr world;
    public: std::string modelName;
    public: double centerLat = 0.0;
    public: double centerLon = 0.0;
    public: ignition::math::Vector2d worldSize;
    public: unsigned int zoom = 21u;
    public: unsigned int tileSizePx = 640u; 
    public: std::string apiKey;
    public: std::vector<std::string> mapTileFilenames;


    /// \brief Pointer to a node for communication.
    public: transport::NodePtr node;

    /// \brief Factory publisher.
    public: transport::PublisherPtr factoryPub;
  };
}

using namespace gazebo;


GZ_REGISTER_WORLD_PLUGIN(StaticMapPlugin)


/////////////////////////////////////////////////
size_t WriteData(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
  size_t written;
  written = fwrite(ptr, size, nmemb, stream);
  return written;
}


/////////////////////////////////////////////////
bool GetStaticMap(const std::string &_url, const std::string &_outputFile)
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

const static unsigned int TILE_SIZE = 256;
ignition::math::Vector2d pixelOrigin(TILE_SIZE / 2, TILE_SIZE / 2);
double pixelsPerLonDegree = TILE_SIZE / 360.0;
double pixelsPerLonRadian = TILE_SIZE / (2.0 * M_PI);



/////////////////////////////////////////////////
ignition::math::Vector2d LatLonToPoint(
    const ignition::math::SphericalCoordinates &_latLon)
{
	ignition::math::Vector2d point;
	ignition::math::Vector2d origin = pixelOrigin;
	
	point.X() = origin.X() + _latLon.LongitudeReference().Degree() * 
      pixelsPerLonDegree;
	
	// Truncating to 0.9999 effectively limits latitude to 89.189. This is
	// about a third of a tile past the edge of the world tile.
	double siny = std::min(std::max(std::sin(
      _latLon.LatitudeReference().Radian()), -0.9999), 0.9999);
	point.Y() = origin.Y() + 0.5 * std::log((1 + siny) / (1 - siny)) * 
      -pixelsPerLonRadian;
	
	return point;
}

/////////////////////////////////////////////////
ignition::math::SphericalCoordinates PointToLatLon(
    const ignition::math::Vector2d &_point)
{
	ignition::math::Vector2d origin = pixelOrigin;

  ignition::math::SphericalCoordinates latLon;
  ignition::math::Angle lonAngle;
  ignition::math::Angle latAngle;

  double lon = (_point.X() - origin.X()) / pixelsPerLonDegree;
  double latRadians = (_point.Y() - origin.Y()) / -pixelsPerLonRadian;

  lonAngle.Degree(lon);
  latAngle.Radian(2 * std::atan(std::exp(latRadians)) - M_PI / 2.0);	

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

  this->dataPtr->apiKey = _sdf->Get<std::string>("api_key");
  this->dataPtr->centerLat = _sdf->Get<double>("center_lat");
  this->dataPtr->centerLon = _sdf->Get<double>("center_lon");
  this->dataPtr->worldSize = _sdf->Get<ignition::math::Vector2d>("world_size");
  if (_sdf->HasElement("model_name"))
    this->dataPtr->modelName = _sdf->Get<std::string>("model_name");
  else
  {
    // generate name based on input
    std::stringstream name;
    name << "tile_" << std::setprecision(12) << this->dataPtr->centerLat << "_" 
         << this->dataPtr->centerLon
         << "_" << this->dataPtr->worldSize.X() << "_" 
         << this->dataPtr->worldSize.Y();
    this->dataPtr->modelName = name.str();
  }
}

/////////////////////////////////////////////////
void StaticMapPlugin::Init()
{
  // check if model exists locally
  auto basePath = common::SystemPaths::Instance()->GetLogPath() / 
        boost::filesystem::path("map_tiles");

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->factoryPub =
      this->dataPtr->node->Advertise<msgs::Factory>("~/factory");

  boost::filesystem::path modelPath = basePath / this->dataPtr->modelName;
  if (common::exists(modelPath.string()))
  {
    // add to gazebo model path so model can be found
    common::SystemPaths::Instance()->AddModelPaths(basePath.string());
    gzwarn << "Model: '" << this->dataPtr->modelName << "' exists! " 
           << "Spawning existing model..";
    // this->dataPtr->SpawnModel(uri);
    return;
  }

  // create model dir structure
  boost::filesystem::path scriptsPath(modelPath / "materials" / "scripts");
  boost::filesystem::create_directories(scriptsPath);
  boost::filesystem::path texturesPath(modelPath / "materials" / "textures");
  boost::filesystem::create_directories(texturesPath);

  std::vector<std::string> tiles = this->dataPtr->DownloadMapTiles(
      this->dataPtr->centerLat,
      this->dataPtr->centerLon, 
      this->dataPtr->zoom, 
      this->dataPtr->tileSizePx,
      this->dataPtr->worldSize,
      this->dataPtr->apiKey,
      texturesPath.string());

  // assume square model for now
  unsigned int xNumTiles = std::sqrt(tiles.size());
  unsigned int yNumTiles = xNumTiles;

  double tileWorldSize = this->dataPtr->GroundResolution(
    this->dataPtr->centerLat / 180 * M_PI, this->dataPtr->zoom) 
    * this->dataPtr->tileSizePx;
  this->dataPtr->CreateMapTileModel(
      this->dataPtr->modelName, tileWorldSize,
      xNumTiles, yNumTiles, tiles, modelPath.string());
}

/////////////////////////////////////////////////
double StaticMapPluginPrivate::GroundResolution(const double _lat, 
    const unsigned int _zoom)
{
  double earthEquatorialRadius = 6378137;
  double metersPerPx = 2 * M_PI * earthEquatorialRadius * 
      std::cos(_lat) / (TILE_SIZE * std::pow(2, _zoom));
  return metersPerPx;
}

/////////////////////////////////////////////////
std::vector<std::string> StaticMapPluginPrivate::DownloadMapTiles(
    const double _centerLat, const double _centerLon, 
    const double _zoom, const unsigned int _tileSizePx,
    const ignition::math::Vector2d &_worldSize, const std::string &_apiKey,
    const std::string &_saveLocation)
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
  double scale = std::pow(2, this->zoom);

  ignition::math::Vector2d centerPx = 
      LatLonToPoint(centerLatLon) * scale;

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
      auto latLon = PointToLatLon(point);    

      // download tile image
      std::stringstream query;
      query << "?center="
            << std::setprecision(12)
            << latLon.LatitudeReference().Degree() << ","
            << latLon.LongitudeReference().Degree()
            << "&zoom=" << _zoom
            << "&size=" << _tileSizePx << "x" << _tileSizePx
            << "&maptype=satellite"
            << "&key=" << _apiKey;
      std::string fullURL = url + query.str();
      std::stringstream filename;
      filename << "tile_"
               << std::setprecision(12) << latLon.LatitudeReference().Degree() << "_" 
               << latLon.LongitudeReference().Degree() << ".png";
      std::string fullPath = _saveLocation + "/" + filename.str();
      GetStaticMap(fullURL, fullPath);
      //std::cerr << " outputFile " << filename.str() << std::endl;
      //std::cerr << " query " << query.str() << std::endl;
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
void StaticMapPluginPrivate::CreateMapTileModel(
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
//        "        filtering anistropic\n"
//        "        max_anisotropy 16\n"
        "      }\n"
        "    }\n"
        "  }\n"
        "}\n\n";
    }
  }
  // std::cerr << " ======== material script: " << std::endl;
  // std::cerr << materialScriptStr.str() << std::endl;

  boost::filesystem::path scriptFilePath(_modelPath);
  scriptFilePath = scriptFilePath / "materials" / "scripts" 
      / "map_tiles.material";
  std::ofstream scriptFile;
  scriptFile.open(scriptFilePath.string().c_str());
  if (!scriptFile.is_open())
  {
    gzerr << "Couldn't open file for writing: " << scriptFilePath.string() << std::endl;
    return;
  }
  scriptFile << materialScriptStr.str();
  scriptFile.close();

  // create model.sdf file
  double sizeX = _tileWorldSize;
  double sizeY = _tileWorldSize;

  ignition::math::Vector2d center(0, 0);
  double halfTileWorldWidth = sizeX * std::floor(_xNumTiles / 2);
  double halfTileWorldHeight = sizeY * std::floor(_yNumTiles / 2);
  double x = center.X() - halfTileWorldWidth;
  double y = center.Y() + halfTileWorldHeight;
  double halfTileWorldSizeX = sizeX / 2.0;
  double halfTileWorldSizeY = sizeY / 2.0;
  if (_xNumTiles % 2 == 0u)
    x += halfTileWorldSizeX;
  if (_yNumTiles % 2 == 0u)
    y -= halfTileWorldSizeY;
  double startx = x;

  // rotate around z to line up textures 
  ignition::math::Vector3d rot(0, 0, M_PI / 2.0);

  std::stringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>\n"
    "<model name='" << _name << "'>\n"
    "  <static>true</static>\n"
    "  <link name='link'>\n";
  for (unsigned int i = 0; i < _yNumTiles; ++i)
  {
    for (unsigned int j = 0; j < _xNumTiles; ++j)
    {
      newModelStr <<
        "    <collision name='collision" << i << "_" << j <<"'>\n"
        "      <pose>" << x << " " << y << " 0.0 " << rot << "</pose>\n"
        "      <geometry>\n"
        "        <box>\n"
        "          <size>" << sizeX << " " << sizeY << " " << 1 << "</size>\n"
        "        </box>\n"
        "      </geometry>\n"
        "    </collision>\n"
        "    <visual name='visual" << i << "_" << j <<"'>\n"
        "      <pose>" << x << " " << y << " 0.0 " << rot << "</pose>\n"
        "      <geometry>\n"
        "        <box>\n"
        "          <size>" << sizeX << " " << sizeY << " " << 1 << "</size>\n"
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

 
  //std::cerr << " ===== newModelStr " << std::endl;
  //std::cerr << newModelStr.str() << std::endl;

  boost::filesystem::path modelSDFFilePath(_modelPath);
  modelSDFFilePath /= "model.sdf";
  std::ofstream modelSDFFile;
  modelSDFFile.open(modelSDFFilePath.string().c_str());
  if (!modelSDFFile.is_open())
  {
    gzerr << "Couldn't open file for writing: " << modelSDFFilePath.string() << std::endl;
    return;
  }
  modelSDFFile << newModelStr.str();
  modelSDFFile.close();

  // this->world->InsertModelString(newModelStr.str());

  // create model.config file
  std::ostringstream modelConfigStr;
  modelConfigStr << "<?xml version=\"1.0\"?>\n"
  << "<model>\n"
  <<   "<name>" << _name << "</name>\n"
  <<   "<version>1.0</version>\n"
  <<   "<sdf version=\"" << SDF_VERSION << "\">model.sdf</sdf>\n"
  <<   "<author>\n"
  <<     "<name>gazebo</name>\n"
  <<     "<email></email>\n"
  <<   "</author>\n"
  <<   "<description>Made with Gazebo Static Map Plugin</description>\n"
  << "</model>";

  // std::cerr << " ===== modelConfigStr " << std::endl;
  // std::cerr << modelConfigStr.str() << std::endl;

  boost::filesystem::path modelConfigFilePath(_modelPath);
  modelConfigFilePath /= "model.config";
  std::ofstream modelConfigFile;
  modelConfigFile.open(modelConfigFilePath.string().c_str());
  if (!modelConfigFile.is_open())
  {
    gzerr << "Couldn't open file for writing: " 
        << modelConfigFilePath.string() << std::endl;
    return;
  }
  modelConfigFile << modelConfigStr.str();
  modelConfigFile.close();

  // publish to factory topic to create the model
  // add to gazebo model path so model can be found
  boost::filesystem::path modelPath(_modelPath);
  common::SystemPaths::Instance()->AddModelPaths(modelPath.parent_path().string());
  msgs::Factory msg;
  msg.set_sdf_filename("model://" + _name);
  // this->factoryPub->Publish(msg);
}

