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

#include <tinyxml.h>
#include <sstream>
#include <boost/algorithm/string.hpp>

#include "math/Vector2d.hh"
#include "math/Vector3.hh"
#include "common/Console.hh"
#include "common/Mesh.hh"
#include "common/ColladaLoader.hh"

using namespace gazebo;
using namespace common;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
ColladaLoader::ColladaLoader()
  : MeshLoader()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
ColladaLoader::~ColladaLoader()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load a mesh
Mesh *ColladaLoader::Load( const std::string &_filename )
{
  TiXmlDocument xmlDoc;
  TiXmlElement *geometriesXML;

  Mesh *mesh = new Mesh();

  if (!xmlDoc.LoadFile(_filename))
    gzerr << "Unable to load collada file[" << _filename << "]\n";

  this->colladaXml = xmlDoc.FirstChildElement("COLLADA");
  if (!this->colladaXml)
    gzerr << "Missing COLLADA tag\n";

  if (std::string(this->colladaXml->Attribute("version")) != "1.4.0" &&
      std::string(this->colladaXml->Attribute("version")) != "1.4.1")
    gzerr << "Invalid collada file. Must be version 1.4.0 or 1.4.1\n";

  geometriesXML = this->colladaXml->FirstChildElement("library_geometries");
  if (!geometriesXML)
    gzerr << "Missing <library_geometries>\n";
  this->LoadGeometries(geometriesXML, mesh);
  
  return mesh;
}

void ColladaLoader::LoadGeometries(TiXmlElement *xml, Mesh *mesh)
{
  TiXmlElement *geomXml = xml->FirstChildElement("geometry");
  while (geomXml)
  {
    std::cout << "Geom Id[" << geomXml->Attribute("id") << "]\n";
    TiXmlElement *meshXml = geomXml->FirstChildElement("mesh");
    TiXmlElement *trianglesXml = meshXml->FirstChildElement("triangles");

    TiXmlElement *trianglesInputXml = trianglesXml->FirstChildElement("input");

    std::vector<math::Vector3> verts;
    std::vector<math::Vector3> norms;
    std::vector<math::Vector2d> texcoords;

    std::map< int,std::string > inputs;
    while (trianglesInputXml)
    {
      std::string semantic = trianglesInputXml->Attribute("semantic");
      std::string source = trianglesInputXml->Attribute("source");
      std::string offset = trianglesInputXml->Attribute("offset");
      if (semantic == "VERTEX")
        this->GetVertices(source, verts);
      else if (semantic == "NORMAL")
        this->GetNormals(source, norms);
      else if (semantic == "TEXCOORD")
        this->GetTexCoords(source, texcoords);

      inputs[0] = semantic;

      trianglesInputXml = trianglesInputXml->NextSiblingElement("input");
    }

    TiXmlElement *pXml = trianglesXml->FirstChildElement("p");
    std::string pStr = pXml->GetText();
    std::istringstream iss(pStr);
    do
    {
      math::Vector2d vec;
      int value;
      for (std::map<int,std::string>::iterator iter = inputs.begin();
          iter != inputs.end(); iter++)
      {
        iss >> value;
        if (iter->second == "VERTEX")
          subMesh->AddIndex(value);
        else if (iter->second == "NORMAL")
          subMesh->Add

      }
    } while (iss);




    //mesh->AddSubMesh(subMesh);

    geomXml = geomXml->NextSiblingElement("geometry");
  }
}

TiXmlElement *ColladaLoader::GetElementId(TiXmlElement *_parent, 
                                          const std::string &_id)
{
  if (_parent->Attribute("id") && std::string("#")+_parent->Attribute("id") == _id)
    return _parent;
  
  TiXmlElement *elem = _parent->FirstChildElement();
  while (elem)
  {
    TiXmlElement *result = this->GetElementId(elem, _id);
    if (result)
      return result;

    elem = elem->NextSiblingElement();
  }

  return NULL;
}

void ColladaLoader::GetVertices(const std::string &_id, 
                                std::vector<math::Vector3> &_values)
{
  TiXmlElement *verticesXml = this->GetElementId(this->colladaXml, _id);
  if (!verticesXml)
    gzerr << "Unable to find vertices[" << _id << "] in collada file\n";

  TiXmlElement *inputXml = verticesXml->FirstChildElement("input");
  while (inputXml)
  {
    std::string verticesId = inputXml->Attribute("source");
    TiXmlElement *sourceXml = this->GetElementId(this->colladaXml, verticesId);
    TiXmlElement *floatArrayXml = sourceXml->FirstChildElement("float_array");
    if (!floatArrayXml)
      gzerr << "Vertex source missing float_array element\n";
    std::string valueStr = floatArrayXml->GetText();
    std::istringstream iss(valueStr);
    do
    {
      math::Vector3 vec;
      iss >> vec.x >> vec.y >> vec.z;
      if (iss)
        _values.push_back(vec);
    } while (iss);

    inputXml = inputXml->NextSiblingElement("input");
  }
}

void ColladaLoader::GetNormals(const std::string &_id, 
                               std::vector<math::Vector3> &_values)
{
  TiXmlElement *normalsXml = this->GetElementId(this->colladaXml, _id);
  if (!normalsXml)
    gzerr << "Unable to find normals[" << _id << "] in collada file\n";

  TiXmlElement *floatArrayXml = normalsXml->FirstChildElement("float_array");
  if (!floatArrayXml)
    gzerr << "Normal source missing float_array element\n";

  std::string valueStr = floatArrayXml->GetText();
  std::istringstream iss(valueStr);
  do
  {
    math::Vector3 vec;
    iss >> vec.x >> vec.y >> vec.z;
    if (iss)
      _values.push_back(vec);
  } while (iss);

}

void ColladaLoader::GetTexCoords(const std::string &_id, 
                               std::vector<math::Vector2d> &_values)
{
  TiXmlElement *xml = this->GetElementId(this->colladaXml, _id);
  if (!xml)
    gzerr << "Unable to find tex coords[" << _id << "] in collada file\n";

  TiXmlElement *floatArrayXml = xml->FirstChildElement("float_array");
  if (!floatArrayXml)
    gzerr << "Normal source missing float_array element\n";

  std::string valueStr = floatArrayXml->GetText();
  std::istringstream iss(valueStr);
  do
  {
    math::Vector2d vec;
    iss >> vec.x >> vec.y;
    if (iss)
      _values.push_back(vec);
  } while (iss);

}
