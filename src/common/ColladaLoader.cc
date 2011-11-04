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
#include <boost/lexical_cast.hpp>

#include "math/Angle.hh"
#include "math/Vector2d.hh"
#include "math/Vector3.hh"
#include "math/Matrix4.hh"
#include "math/Quaternion.hh"
#include "common/Console.hh"
#include "common/Material.hh"
#include "common/Mesh.hh"
#include "common/ColladaLoader.hh"
#include "common/SystemPaths.hh"

using namespace gazebo;
using namespace common;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
ColladaLoader::ColladaLoader()
  : MeshLoader(), meter(1.0)
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

  this->path.clear();
  if (_filename.rfind('/') != std::string::npos)
  {
    this->path = _filename.substr(0,_filename.rfind('/'));
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(this->path.c_str());
  }

  if (!xmlDoc.LoadFile(_filename))
    gzerr << "Unable to load collada file[" << _filename << "]\n";

  this->colladaXml = xmlDoc.FirstChildElement("COLLADA");
  if (!this->colladaXml)
    gzerr << "Missing COLLADA tag\n";

  if (std::string(this->colladaXml->Attribute("version")) != "1.4.0" &&
      std::string(this->colladaXml->Attribute("version")) != "1.4.1")
    gzerr << "Invalid collada file. Must be version 1.4.0 or 1.4.1\n";

  TiXmlElement *assetXml = this->colladaXml->FirstChildElement("asset");
  if (assetXml)
  {
    TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
    if (unitXml && unitXml->Attribute("meter"))
      this->meter = boost::lexical_cast<double>(unitXml->Attribute("meter"));
  }

  Mesh *mesh = new Mesh();
  this->LoadScene(mesh);

  return mesh;
}

void ColladaLoader::LoadScene( Mesh *_mesh )
{
  TiXmlElement *sceneXml = this->colladaXml->FirstChildElement("scene");
  std::string sceneURL = sceneXml->FirstChildElement("instance_visual_scene")->Attribute("url");

  TiXmlElement *visSceneXml = this->GetElementId("visual_scene", sceneURL);

  if (!visSceneXml)
    gzerr << "Unable to find visual_scene id='" << sceneURL << "'\n";

  TiXmlElement *nodeXml = visSceneXml->FirstChildElement("node");
  while (nodeXml)
  {
    this->LoadNode( nodeXml , _mesh, math::Matrix4::IDENTITY);
    nodeXml = nodeXml->NextSiblingElement("node");
  }
}
void ColladaLoader::LoadGeometry(TiXmlElement *_xml, 
                                 const math::Matrix4 &_transform, Mesh *_mesh)
{
  TiXmlElement *meshXml = _xml->FirstChildElement("mesh");
  TiXmlElement *childXml;

  childXml = meshXml->FirstChildElement("triangles");
  while (childXml)
  {
    this->LoadTriangles(childXml, _transform, _mesh);
    childXml = childXml->NextSiblingElement("triangles");
  }

  childXml = meshXml->FirstChildElement("lines");
  while (childXml)
  {
    this->LoadLines(childXml, _transform, _mesh);
    childXml = childXml->NextSiblingElement("lines");
  }
}

TiXmlElement *ColladaLoader::GetElementId( const std::string &_name, const std::string &_id)
{
  return this->GetElementId(this->colladaXml, _name, _id);
}

TiXmlElement *ColladaLoader::GetElementId(TiXmlElement *_parent, 
                                          const std::string &_name,
                                          const std::string &_id)
{
  std::string id = _id;
  if (id.find("#") != std::string::npos)
    id.erase(id.find("#"),1);

  if ( (id.empty() && _parent->Value() == _name) ||
       (_parent->Attribute("id")  && _parent->Attribute("id") == id) ||
       (_parent->Attribute("sid") && _parent->Attribute("sid") == id) )
  {
    return _parent;
  }

  TiXmlElement *elem = _parent->FirstChildElement();
  while (elem)
  {
    TiXmlElement *result = this->GetElementId(elem, _name, _id);
    if (result)
      return result;

    elem = elem->NextSiblingElement();
  }

  return NULL;
}

math::Matrix4 ColladaLoader::LoadNodeTransform(TiXmlElement *_elem)
{
  math::Matrix4 transform(math::Matrix4::IDENTITY);

  if (_elem->FirstChildElement("matrix"))
  {
    std::string matrixStr = _elem->FirstChildElement("matrix")->GetText();
    std::istringstream iss(matrixStr);
    std::vector<double> values(16);
    for (unsigned int i=0; i < 16; i++)
      iss >> values[i];
    transform.Set(values[0], values[1], values[2], values[3],
                  values[4], values[5], values[6], values[7],
                  values[8], values[9], values[10], values[11],
                  values[12], values[13], values[14], values[15]);
  }
  else
  {
    if (_elem->FirstChildElement("translate"))
    {
      std::string transStr = _elem->FirstChildElement("translate")->GetText();
      math::Vector3 translate;
      translate = boost::lexical_cast<math::Vector3>(transStr);
      //translate *= this->meter;
      transform.SetTranslate( translate );
    }

    TiXmlElement *rotateXml = _elem->FirstChildElement("rotate");
    while (rotateXml)
    {
      math::Matrix3 mat;
      math::Vector3 axis;
      double angle;

      std::string rotateStr = rotateXml->GetText();
      std::istringstream iss(rotateStr);

      iss >> axis.x >> axis.y >> axis.z;
      iss >> angle;
      mat.SetFromAxis(axis,DTOR(angle));

      transform = transform * mat;

      rotateXml = rotateXml->NextSiblingElement("rotate");
    }

    if (_elem->FirstChildElement("scale"))
    {
      std::string scaleStr = _elem->FirstChildElement("scale")->GetText();
      math::Vector3 scale;
      scale = boost::lexical_cast<math::Vector3>(scaleStr);
      math::Matrix4 scaleMat;
      scaleMat.SetScale( scale );
      transform = transform * scaleMat;
    }
  }

  return transform;
}

void ColladaLoader::LoadVertices(const std::string &_id, 
                                 const math::Matrix4 &_transform,
                                 std::vector<math::Vector3> &_verts,
                                 std::vector<math::Vector3> &_norms)
{
  TiXmlElement *verticesXml = this->GetElementId(this->colladaXml, "vertices", _id);
  if (!verticesXml)
    gzerr << "Unable to find vertices[" << _id << "] in collada file\n";

  TiXmlElement *inputXml = verticesXml->FirstChildElement("input");
  while (inputXml)
  {
    std::string semantic = inputXml->Attribute("semantic");
    std::string sourceStr = inputXml->Attribute("source");
    if (semantic == "NORMAL")
    {
      this->LoadNormals(sourceStr, _norms);
    }
    else if (semantic == "POSITION")
    {
      this->LoadPositions(sourceStr, _transform, _verts);
    }

    inputXml = inputXml->NextSiblingElement("input");
  }
}

void ColladaLoader::LoadPositions(const std::string &_id,
                                  const math::Matrix4 &_transform,
                                  std::vector<math::Vector3> &_values)
{
  TiXmlElement *sourceXml = this->GetElementId("source", _id);
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
    {
      //vec = vec * this->meter;
      vec = _transform * vec;
      _values.push_back(vec);
    }
  } while (iss);

}

void ColladaLoader::LoadNormals(const std::string &_id, 
                               std::vector<math::Vector3> &_values)
{
  TiXmlElement *normalsXml = this->GetElementId("source", _id);
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
    {
      vec.Normalize();
      _values.push_back(vec);
    }
  } while (iss);

}

void ColladaLoader::LoadTexCoords(const std::string &_id, 
                               std::vector<math::Vector2d> &_values)
{
  TiXmlElement *xml = this->GetElementId("source", _id);
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
    {
      vec.y = 1.0 - vec.y;
      _values.push_back(vec);
    }
  } while (iss);

}

Material *ColladaLoader::LoadMaterial(const std::string &_name)
{
  Material *mat = new Material();

  TiXmlElement *matXml = this->GetElementId("material", _name);
  std::string effectName = matXml->FirstChildElement("instance_effect")->Attribute("url");
  TiXmlElement *effectXml = this->GetElementId("effect", effectName);

  TiXmlElement *commonXml = effectXml->FirstChildElement("profile_COMMON");
  if (commonXml)
  {
    TiXmlElement *techniqueXml = commonXml->FirstChildElement("technique");
    TiXmlElement *lambertXml = techniqueXml->FirstChildElement("lambert");

    TiXmlElement *phongXml = techniqueXml->FirstChildElement("phong");
    TiXmlElement *blinnXml = techniqueXml->FirstChildElement("blinn");
    if (lambertXml)
    {
      this->LoadColorOrTexture(lambertXml, "ambient", mat);
      this->LoadColorOrTexture(lambertXml, "diffuse", mat);
      this->LoadColorOrTexture(lambertXml, "emission", mat);
      if (lambertXml->FirstChildElement("transparency"))
      {
        mat->SetTransparency( 
            this->LoadFloat(lambertXml->FirstChildElement("transparency")) );
      }

      if (lambertXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = lambertXml->FirstChildElement("transparent");
        this->LoadTransparent( transXml, mat );
      }
    }
    else if (phongXml)
    {
      this->LoadColorOrTexture(phongXml, "ambient", mat);
      this->LoadColorOrTexture(phongXml, "diffuse", mat);
      this->LoadColorOrTexture(phongXml, "emission", mat);
      this->LoadColorOrTexture(phongXml, "specular", mat);
      if (phongXml->FirstChildElement("shininess"))
        mat->SetShininess( 
            this->LoadFloat(phongXml->FirstChildElement("shininess")) );

      if (phongXml->FirstChildElement("transparency"))
        mat->SetTransparency( 
            this->LoadFloat(phongXml->FirstChildElement("transparency")) );
      if (phongXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = phongXml->FirstChildElement("transparent");
        this->LoadTransparent( transXml, mat );
      }

    }
    else if (blinnXml)
    {
      this->LoadColorOrTexture(blinnXml, "ambient", mat);
      this->LoadColorOrTexture(blinnXml, "diffuse", mat);
      this->LoadColorOrTexture(blinnXml, "emission", mat);
      this->LoadColorOrTexture(blinnXml, "specular", mat);
      if (blinnXml->FirstChildElement("shininess"))
        mat->SetShininess( 
            this->LoadFloat(blinnXml->FirstChildElement("shininess")) );

      if (blinnXml->FirstChildElement("transparency"))
        mat->SetTransparency( 
            this->LoadFloat(blinnXml->FirstChildElement("transparency")) );
      if (blinnXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = blinnXml->FirstChildElement("transparent");
        this->LoadTransparent( transXml, mat );
      }
    }
  }

  TiXmlElement *glslXml = effectXml->FirstChildElement("profile_GLSL");
  if (glslXml)
    gzerr << "profile_GLSL unsupported\n";

  TiXmlElement *cgXml = effectXml->FirstChildElement("profile_CG");
  if (cgXml)
    gzerr << "profile_CG unsupported\n";

  return mat;
}

void ColladaLoader::LoadColorOrTexture(TiXmlElement *_elem, 
                                  const std::string &_type, Material *_mat)
{
  if (!_elem || !_elem->FirstChildElement(_type))
    return;

  TiXmlElement *typeElem = _elem->FirstChildElement(_type);

  if (typeElem->FirstChildElement("color"))
  {
    std::string colorStr = typeElem->FirstChildElement("color")->GetText();
    Color color = boost::lexical_cast<Color>(colorStr);
    if (_type == "diffuse")
      _mat->SetDiffuse( color );
    else if (_type == "ambient")
      _mat->SetAmbient( color );
    else if (_type == "emission")
      _mat->SetEmissive( color );
  }
  else if (typeElem->FirstChildElement("texture"))
  {
    std::string textureName = typeElem->FirstChildElement("texture")->Attribute("texture");
    TiXmlElement *textureXml = this->GetElementId("newparam", textureName);
    TiXmlElement *sampler = textureXml->FirstChildElement("sampler2D");
    std::string sourceName = sampler->FirstChildElement("source")->GetText();
    TiXmlElement *sourceXml = this->GetElementId("newparam", sourceName);
    TiXmlElement *surfaceXml = sourceXml->FirstChildElement("surface");
    if (surfaceXml->FirstChildElement("init_from"))
    {
      TiXmlElement *imageXml = this->GetElementId("image",
          surfaceXml->FirstChildElement("init_from")->GetText());
      if (imageXml->FirstChildElement("init_from"))
      {
        std::string imgFile = imageXml->FirstChildElement("init_from")->GetText();
        _mat->SetTextureImage(imgFile );
      }
    }
  }
}

void ColladaLoader::LoadTriangles( TiXmlElement *_trianglesXml, 
                                   const math::Matrix4 &_transform,
                                   Mesh *_mesh)
{
  SubMesh *subMesh = new SubMesh;
  bool combinedVertNorms = false;

  subMesh->SetPrimitiveType( SubMesh::TRIANGLES );

  if (_trianglesXml->Attribute("material"))
  {
    std::map<std::string, std::string>::iterator iter;
    std::string matStr = _trianglesXml->Attribute("material");

    iter = this->materialMap.find(matStr);
    if (iter != this->materialMap.end())
      matStr = iter->second;

    unsigned int matIndex = _mesh->AddMaterial( this->LoadMaterial(matStr) );
    subMesh->SetMaterialIndex( matIndex );
  }

  TiXmlElement *trianglesInputXml = _trianglesXml->FirstChildElement("input");

  std::vector<math::Vector3> verts;
  std::vector<math::Vector3> norms;
  std::vector<math::Vector2d> texcoords;

  std::map< std::string,int > inputs;
  while (trianglesInputXml)
  {
    std::string semantic = trianglesInputXml->Attribute("semantic");
    std::string source = trianglesInputXml->Attribute("source");
    std::string offset = trianglesInputXml->Attribute("offset");
    if (semantic == "VERTEX")
    {
      unsigned int count = norms.size();
      this->LoadVertices(source, _transform, verts, norms);
      if (norms.size() > count)
        combinedVertNorms = true;
    }
    else if (semantic == "NORMAL")
    {
      this->LoadNormals(source, norms);
      combinedVertNorms = false;
    }
    else if (semantic == "TEXCOORD")
      this->LoadTexCoords(source, texcoords);

    inputs[semantic] = boost::lexical_cast<int>(offset);

    trianglesInputXml = trianglesInputXml->NextSiblingElement("input");
  }

  TiXmlElement *pXml = _trianglesXml->FirstChildElement("p");
  std::string pStr = pXml->GetText();
  std::istringstream iss(pStr);

  std::vector<math::Vector3> vertNorms(verts.size());
  std::vector<int> vertNormsCounts(verts.size());
  std::fill(vertNormsCounts.begin(), vertNormsCounts.end(), 0);

  do
  {
    math::Vector2d vec;
    int *values = new int[inputs.size()];
    for (unsigned int i=0; i < inputs.size(); i++)
      iss >> values[i];

    if (!iss)
    {
      delete [] values;
      break;
    }

    for (std::map<std::string,int>::iterator iter = inputs.begin();
        iter != inputs.end(); iter++)
    {
      if (iter->first == "VERTEX")
      {
        subMesh->AddVertex(verts[values[iter->second]]);
        subMesh->AddIndex(subMesh->GetVertexCount()-1);
        if (combinedVertNorms)
          subMesh->AddNormal(norms[values[iter->second]]);
      }
      else if (iter->first == "NORMAL")
      {
        subMesh->AddNormal(norms[values[iter->second]]);
      }
      else if (iter->first == "TEXCOORD")
      {
        subMesh->AddTexCoord( texcoords[values[iter->second]].x,
            texcoords[values[iter->second]].y);
      }
      /*else
        gzerr << "Unhandled semantic[" << iter->first << "]\n";
        */
    }
    delete [] values;
  } while (iss);

  _mesh->AddSubMesh(subMesh);
}

void ColladaLoader::LoadLines( TiXmlElement *_xml, 
                               const math::Matrix4 &_transform,
                               Mesh *_mesh)
{
  SubMesh *subMesh = new SubMesh;
  subMesh->SetPrimitiveType( SubMesh::LINES );

  TiXmlElement *inputXml = _xml->FirstChildElement("input");
  std::string semantic = inputXml->Attribute("semantic");
  std::string source =   inputXml->Attribute("source");

  std::vector<math::Vector3> verts;
  std::vector<math::Vector3> norms;
  this->LoadVertices(source, _transform, verts,norms);

  TiXmlElement *pXml = _xml->FirstChildElement("p");
  std::string pStr = pXml->GetText();
  std::istringstream iss(pStr);

  do
  {
    int a,b;
    iss >> a >> b;

    if (!iss)
      break;
    subMesh->AddVertex( verts[a] );
    subMesh->AddIndex( subMesh->GetVertexCount() - 1 );
    subMesh->AddVertex( verts[b] );
    subMesh->AddIndex( subMesh->GetVertexCount() - 1 );

  } while (iss);

  _mesh->AddSubMesh(subMesh);
}



void ColladaLoader::LoadNode(TiXmlElement *_elem, Mesh *_mesh, 
                             const math::Matrix4 &_transform)
{
  TiXmlElement *nodeXml;
  TiXmlElement *instGeomXml;

  math::Matrix4 transform = this->LoadNodeTransform( _elem );
  transform = _transform * transform;

  nodeXml = _elem->FirstChildElement("node");
  while (nodeXml)
  {
    this->LoadNode(nodeXml,_mesh, transform);
    nodeXml = nodeXml->NextSiblingElement("node");
  }

  if (_elem->FirstChildElement("instance_node"))
  {
    std::string nodeURLStr = _elem->FirstChildElement("instance_node")->Attribute("url");

    nodeXml = this->GetElementId("node", nodeURLStr);
    if (!nodeXml)
    {
      gzerr << "Unable to find node[" << nodeURLStr << "]\n";
      return;
    }
    this->LoadNode(nodeXml, _mesh, transform);
    return;
  }
  else 
    nodeXml = _elem;

  instGeomXml = nodeXml->FirstChildElement("instance_geometry");
  while (instGeomXml)
  {
    std::string geomURL = instGeomXml->Attribute("url");
    TiXmlElement *geomXml = this->GetElementId("geometry", geomURL);

    this->materialMap.clear();
    TiXmlElement *bindMatXml, *techniqueXml, *matXml;
    bindMatXml = instGeomXml->FirstChildElement("bind_material");
    while (bindMatXml)
    {
      if ((techniqueXml = bindMatXml->FirstChildElement("technique_common")))
      {
        matXml = techniqueXml->FirstChildElement("instance_material");
        while (matXml)
        {
          std::string symbol = matXml->Attribute("symbol");
          std::string target = matXml->Attribute("target");
          this->materialMap[symbol] = target;
          matXml = matXml->NextSiblingElement("instance_material");
        }
      }
      bindMatXml = bindMatXml->NextSiblingElement("bind_material");
    }

    this->LoadGeometry(geomXml, transform, _mesh);
    instGeomXml = instGeomXml->NextSiblingElement("instance_geometry");
  }

}

float ColladaLoader::LoadFloat(TiXmlElement *_elem)
{
  float value = 0;

  if (_elem->FirstChildElement("float"))
  {
    value = boost::lexical_cast<float>(
                                 _elem->FirstChildElement("float")->GetText());
  }

  return value;
}

void ColladaLoader::LoadTransparent( TiXmlElement *_elem, Material * /*_mat*/ )
{
  const char *opaque = _elem->Attribute("opaque");
  if (!opaque)
    gzerr << "No Opaque set\n";

  const char *colorStr =_elem->FirstChildElement("color")->GetText();
  if (!colorStr)
    gzerr << "No color string\n";

  /*std::string opaque = 
  std::string colorStr = _elem->FirstChildElement("color")->GetText();
  Color color = boost::lexical_cast<Color>(colorStr);
  */

  //double srcFactor, dstFactor;

  /*if (opaque == "RGB_ZERO")
  {
    //srcFactor = color.R() * _mat->GetTransparency();
    //dstFactor = 1.0 - color.R() * _mat->GetTransparency();
  }
  else if (opaque == "A_ONE")
  {
   // _mat->SetBlendFactors( ONE_MINUS_SRC_ALPHA, SRC_ALPHA);
    //srcFactor = 1.0 - color.A() * _mat->GetTransparency();
    //dstFactor = color.A() * _mat->GetTransparency();
  }
  */

  //_mat->SetBlendFactors( ONE_MINUS_SRC_ALPHA, SRC_ALPHA);
  //_mat->SetBlendFactors(srcFactor, dstFactor);
}
