/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <boost/filesystem.hpp>
#include <ignition/math/Vector3.hh>
#include <tinyxml2.h>

#include "gazebo/common/Material.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/ColladaExporterPrivate.hh"
#include "gazebo/common/ColladaExporter.hh"

#ifdef _WIN32
  #define snprintf _snprintf
#endif

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
ColladaExporter::ColladaExporter()
: MeshExporter(), dataPtr(new ColladaExporterPrivate)
{
}

//////////////////////////////////////////////////
ColladaExporter::~ColladaExporter()
{
  delete this->dataPtr;
  this->dataPtr = 0;
}

//////////////////////////////////////////////////
void ColladaExporter::Export(const Mesh *_mesh, const std::string &_filename,
    bool _exportTextures)
{
  this->dataPtr->mesh = _mesh;
  this->dataPtr->materialCount = this->dataPtr->mesh->GetMaterialCount();
  this->dataPtr->subMeshCount = this->dataPtr->mesh->GetSubMeshCount();
  this->dataPtr->exportTextures = _exportTextures;

  // File name and path
  unsigned int beginFilename = _filename.rfind("/")+1;

  this->dataPtr->path = _filename.substr(0, beginFilename);
  this->dataPtr->filename = _filename.substr(beginFilename);

  if (this->dataPtr->materialCount != 0 &&
      this->dataPtr->materialCount != this->dataPtr->subMeshCount)
  {
    gzwarn << "Material count [" << this->dataPtr->materialCount <<
        "] different from submesh count [" <<
        this->dataPtr->subMeshCount << "]\n";
  }

  // XML declaration
  auto declarationXml = this->dataPtr->xmlDoc.NewDeclaration();
  this->dataPtr->xmlDoc.LinkEndChild(declarationXml);

  // Collada element
  auto colladaXml = this->dataPtr->xmlDoc.NewElement("COLLADA");
  this->dataPtr->xmlDoc.LinkEndChild(colladaXml);
  colladaXml->SetAttribute("version", "1.4.1");
  colladaXml->SetAttribute("xmlns",
      "http://www.collada.org/2005/11/COLLADASchema");

  // Asset element
  auto assetXml = this->dataPtr->xmlDoc.NewElement("asset");
  this->ExportAsset(assetXml);
  colladaXml->LinkEndChild(assetXml);

  // Library geometries element
  auto libraryGeometriesXml = this->dataPtr->xmlDoc.NewElement("library_geometries");
  this->ExportGeometries(libraryGeometriesXml);
  colladaXml->LinkEndChild(libraryGeometriesXml);

  if (this->dataPtr->materialCount != 0)
  {
    // Library images element
    auto libraryImagesXml = this->dataPtr->xmlDoc.NewElement("library_images");
    int imageCount = this->ExportImages(libraryImagesXml);
    if (imageCount != 0)
    {
      colladaXml->LinkEndChild(libraryImagesXml);
    }

    // Library materials element
    auto libraryMaterialsXml = this->dataPtr->xmlDoc.NewElement("library_materials");
    this->ExportMaterials(libraryMaterialsXml);
    colladaXml->LinkEndChild(libraryMaterialsXml);

    // Library effects element
    auto libraryEffectsXml = this->dataPtr->xmlDoc.NewElement("library_effects");
    this->ExportEffects(libraryEffectsXml);
    colladaXml->LinkEndChild(libraryEffectsXml);
  }

  // Library visual scenes element
  auto libraryVisualScenesXml = this->dataPtr->xmlDoc.NewElement("library_visual_scenes");
  this->ExportVisualScenes(libraryVisualScenesXml);
  colladaXml->LinkEndChild(libraryVisualScenesXml);

  // Scene element
  auto sceneXml = this->dataPtr->xmlDoc.NewElement("scene");
  this->ExportScene(sceneXml);
  colladaXml->LinkEndChild(sceneXml);

  // Save file
  if (this->dataPtr->exportTextures)
  {
    boost::filesystem::create_directories(
        boost::filesystem::path(this->dataPtr->path + this->dataPtr->filename +
        std::string("/meshes/")));
    this->dataPtr->xmlDoc.SaveFile((this->dataPtr->path + this->dataPtr->filename +
        std::string("/meshes/") + this->dataPtr->filename +
        std::string(".dae")).c_str());
  }
  else
  {
    this->dataPtr->xmlDoc.SaveFile((this->dataPtr->path + this->dataPtr->filename +
        std::string(".dae")).c_str());
  }
}

//////////////////////////////////////////////////
void ColladaExporter::ExportAsset(tinyxml2::XMLElement *_assetXml)
{
  auto unitXml = this->dataPtr->xmlDoc.NewElement("unit");
  unitXml->SetAttribute("meter", "1");
  unitXml->SetAttribute("name", "meter");
  _assetXml->LinkEndChild(unitXml);

  auto upAxisXml = this->dataPtr->xmlDoc.NewElement("up_axis");
  upAxisXml->LinkEndChild(this->dataPtr->xmlDoc.NewText("Z_UP"));
  _assetXml->LinkEndChild(upAxisXml);
}

//////////////////////////////////////////////////
void ColladaExporter::ExportGeometrySource(
    const gazebo::common::SubMesh *_subMesh,
    tinyxml2::XMLElement *_meshXml, GeometryType _type, const char *_meshID)
{
  char sourceId[100], sourceArrayId[100];
  std::ostringstream fillData;
  fillData.precision(8);
  fillData << std::fixed;
  int stride;
  unsigned int count = 0;

  if (_type == POSITION)
  {
    snprintf(sourceId, sizeof(sourceId), "%s-Positions", _meshID);
    count = _subMesh->GetVertexCount();
    stride = 3;
    ignition::math::Vector3d vertex;
    for (unsigned int i = 0; i < count; ++i)
    {
      vertex = _subMesh->Vertex(i);
      fillData << vertex.X() << " " << vertex.Y() << " " << vertex.Z() << " ";
    }
  }
  if (_type == NORMAL)
  {
    snprintf(sourceId, sizeof(sourceId), "%s-Normals", _meshID);
    count = _subMesh->GetNormalCount();
    stride = 3;
    ignition::math::Vector3d normal;
    for (unsigned int i = 0; i < count; ++i)
    {
      normal = _subMesh->Normal(i);
      fillData << normal.X() << " " << normal.Y() << " " << normal.Z() << " ";
    }
  }
  if (_type == UVMAP)
  {
    snprintf(sourceId, sizeof(sourceId), "%s-UVMap", _meshID);
    count = _subMesh->GetVertexCount();
    stride = 2;
    ignition::math::Vector2d inTexCoord;
    for (unsigned int i = 0; i < count; ++i)
    {
      inTexCoord = _subMesh->TexCoord(i);
      fillData << inTexCoord.X() << " " << 1-inTexCoord.Y() << " ";
    }
  }
  auto sourceXml = this->dataPtr->xmlDoc.NewElement("source");
  _meshXml->LinkEndChild(sourceXml);
  sourceXml->SetAttribute("id", sourceId);
  sourceXml->SetAttribute("name", sourceId);

  snprintf(sourceArrayId, sizeof(sourceArrayId), "%s-array", sourceId);
  auto floatArrayXml = this->dataPtr->xmlDoc.NewElement("float_array");
  floatArrayXml->SetAttribute("count", count *stride);
  floatArrayXml->SetAttribute("id", sourceArrayId);
  floatArrayXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(
      fillData.str().c_str()));
  sourceXml->LinkEndChild(floatArrayXml);

  auto techniqueCommonXml =
      this->dataPtr->xmlDoc.NewElement("technique_common");
  sourceXml->LinkEndChild(techniqueCommonXml);

  snprintf(sourceArrayId, sizeof(sourceArrayId), "#%s-array", sourceId);
  auto accessorXml = this->dataPtr->xmlDoc.NewElement("accessor");
  accessorXml->SetAttribute("count", count);
  accessorXml->SetAttribute("source", sourceArrayId);
  accessorXml->SetAttribute("stride", stride);
  techniqueCommonXml->LinkEndChild(accessorXml);

  auto paramXml = this->dataPtr->xmlDoc.NewElement("param");
  if (_type == POSITION || _type == NORMAL)
  {
    paramXml->SetAttribute("type", "float");
    paramXml->SetAttribute("name", "X");
    accessorXml->LinkEndChild(paramXml);

    paramXml = this->dataPtr->xmlDoc.NewElement("param");
    paramXml->SetAttribute("type", "float");
    paramXml->SetAttribute("name", "Y");
    accessorXml->LinkEndChild(paramXml);

    paramXml = this->dataPtr->xmlDoc.NewElement("param");
    paramXml->SetAttribute("type", "float");
    paramXml->SetAttribute("name", "Z");
    accessorXml->LinkEndChild(paramXml);
  }
  if (_type == UVMAP)
  {
    paramXml->SetAttribute("type", "float");
    paramXml->SetAttribute("name", "U");
    accessorXml->LinkEndChild(paramXml);

    paramXml = this->dataPtr->xmlDoc.NewElement("param");
    paramXml->SetAttribute("type", "float");
    paramXml->SetAttribute("name", "V");
    accessorXml->LinkEndChild(paramXml);
  }
}

//////////////////////////////////////////////////
void ColladaExporter::ExportGeometries(
    tinyxml2::XMLElement *_libraryGeometriesXml)
{
  for (unsigned int i = 0; i < this->dataPtr->subMeshCount; ++i)
  {
    char meshId[100], materialId[100];
    snprintf(meshId, sizeof(meshId), "mesh_%u", i);
    snprintf(materialId, sizeof(materialId), "material_%u", i);

    auto geometryXml = this->dataPtr->xmlDoc.NewElement("geometry");
    geometryXml->SetAttribute("id", meshId);
    _libraryGeometriesXml->LinkEndChild(geometryXml);

    auto meshXml = this->dataPtr->xmlDoc.NewElement("mesh");
    geometryXml->LinkEndChild(meshXml);

    const gazebo::common::SubMesh *subMesh =
        this->dataPtr->mesh->GetSubMesh(i);

    ExportGeometrySource(subMesh, meshXml, POSITION, meshId);
    ExportGeometrySource(subMesh, meshXml, NORMAL, meshId);
    if (subMesh->GetTexCoordCount() != 0)
    {
      ExportGeometrySource(subMesh, meshXml, UVMAP, meshId);
    }

    char attributeValue[100];

    auto verticesXml = this->dataPtr->xmlDoc.NewElement("vertices");
    meshXml->LinkEndChild(verticesXml);
    snprintf(attributeValue, sizeof(attributeValue), "%s-Vertex", meshId);
    verticesXml->SetAttribute("id", attributeValue);
    verticesXml->SetAttribute("name", attributeValue);

    auto inputXml = this->dataPtr->xmlDoc.NewElement("input");
    verticesXml->LinkEndChild(inputXml);
    inputXml->SetAttribute("semantic", "POSITION");
    snprintf(attributeValue, sizeof(attributeValue), "#%s-Positions", meshId);
    inputXml->SetAttribute("source", attributeValue);

    unsigned int indexCount = subMesh->GetIndexCount();

    auto trianglesXml = this->dataPtr->xmlDoc.NewElement("triangles");
    meshXml->LinkEndChild(trianglesXml);
    trianglesXml->SetAttribute("count", indexCount/3);
    if (this->dataPtr->materialCount != 0)
    {
      trianglesXml->SetAttribute("material", materialId);
    }

    inputXml = this->dataPtr->xmlDoc.NewElement("input");
    trianglesXml->LinkEndChild(inputXml);
    inputXml->SetAttribute("offset", 0);
    inputXml->SetAttribute("semantic", "VERTEX");
    snprintf(attributeValue, sizeof(attributeValue), "#%s-Vertex", meshId);
    inputXml->SetAttribute("source", attributeValue);

    inputXml = this->dataPtr->xmlDoc.NewElement("input");
    trianglesXml->LinkEndChild(inputXml);
    inputXml->SetAttribute("offset", 1);
    inputXml->SetAttribute("semantic", "NORMAL");
    snprintf(attributeValue, sizeof(attributeValue), "#%s-Normals", meshId);
    inputXml->SetAttribute("source", attributeValue);

    if (subMesh->GetTexCoordCount() != 0)
    {
      inputXml = this->dataPtr->xmlDoc.NewElement("input");
      trianglesXml->LinkEndChild(inputXml);
      inputXml->SetAttribute("offset", 2);
      inputXml->SetAttribute("semantic", "TEXCOORD");
      snprintf(attributeValue, sizeof(attributeValue), "#%s-UVMap", meshId);
      inputXml->SetAttribute("source", attributeValue);
    }

    std::ostringstream fillData;
    for (unsigned int j = 0; j < indexCount; ++j)
    {
      fillData << subMesh->GetIndex(j) << " "
               << subMesh->GetIndex(j) << " ";
      if (subMesh->GetTexCoordCount() != 0)
      {
        fillData << subMesh->GetIndex(j) << " ";
      }
    }

    auto pXml = this->dataPtr->xmlDoc.NewElement("p");
    trianglesXml->LinkEndChild(pXml);
    pXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(fillData.str().c_str()));
  }
}

//////////////////////////////////////////////////
int ColladaExporter::ExportImages(tinyxml2::XMLElement *_libraryImagesXml)
{
  int imageCount = 0;
  for (unsigned int i = 0; i < this->dataPtr->materialCount; ++i)
  {
    const gazebo::common::Material *material =
        this->dataPtr->mesh->GetMaterial(i);
    std::string imageString = material->GetTextureImage();

    if (imageString.find("meshes/") != std::string::npos)
    {
      char id[100];
      snprintf(id, sizeof(id), "image_%u", i);

      auto imageXml = this->dataPtr->xmlDoc.NewElement("image");
      imageXml->SetAttribute("id", id);
      _libraryImagesXml->LinkEndChild(imageXml);

      auto initFromXml = this->dataPtr->xmlDoc.NewElement("init_from");
      initFromXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(
        imageString.substr(imageString.find("meshes/")+7).c_str()));
      imageXml->LinkEndChild(initFromXml);

      if (this->dataPtr->exportTextures)
      {
        boost::filesystem::create_directories(boost::filesystem::path(
            this->dataPtr->path + this->dataPtr->filename +
            "/materials/textures"));

        std::ifstream  src(imageString.c_str(), std::ios::binary);
        std::ofstream  dst((this->dataPtr->path + this->dataPtr->filename +
            "/materials/textures" + imageString.substr(
            imageString.rfind("/"))).c_str(), std::ios::binary);
        dst << src.rdbuf();
      }

      imageCount++;
    }
  }

  return imageCount;
}

//////////////////////////////////////////////////
void ColladaExporter::ExportMaterials(
    tinyxml2::XMLElement *_libraryMaterialsXml)
{
  for (unsigned int i = 0; i < this->dataPtr->materialCount; ++i)
  {
    char id[100];
    snprintf(id, sizeof(id), "material_%u", i);

    auto materialXml = this->dataPtr->xmlDoc.NewElement("material");
    materialXml->SetAttribute("id", id);
    _libraryMaterialsXml->LinkEndChild(materialXml);

    snprintf(id, sizeof(id), "#material_%u_fx", i);
    auto instanceEffectXml = this->dataPtr->xmlDoc.NewElement("instance_effect");
    instanceEffectXml->SetAttribute("url", id);
    materialXml->LinkEndChild(instanceEffectXml);
  }
}

//////////////////////////////////////////////////
void ColladaExporter::ExportEffects(tinyxml2::XMLElement *_libraryEffectsXml)
{
  for (unsigned int i = 0; i < this->dataPtr->materialCount; ++i)
  {
    char id[100];
    snprintf(id, sizeof(id), "material_%u_fx", i);

    auto effectXml = this->dataPtr->xmlDoc.NewElement("effect");
    effectXml->SetAttribute("id", id);
    _libraryEffectsXml->LinkEndChild(effectXml);

    auto profileCommonXml = this->dataPtr->xmlDoc.NewElement("profile_COMMON");
    effectXml->LinkEndChild(profileCommonXml);

    // Image
    const gazebo::common::Material *material =
        this->dataPtr->mesh->GetMaterial(i);
    std::string imageString = material->GetTextureImage();

    if (imageString.find("meshes/") != std::string::npos)
    {
      auto newParamXml = this->dataPtr->xmlDoc.NewElement("newparam");
      snprintf(id, sizeof(id), "image_%u_surface", i);
      newParamXml->SetAttribute("sid", id);
      profileCommonXml->LinkEndChild(newParamXml);

      auto surfaceXml = this->dataPtr->xmlDoc.NewElement("surface");
      surfaceXml->SetAttribute("type", "2D");
      newParamXml->LinkEndChild(surfaceXml);

      auto initFromXml = this->dataPtr->xmlDoc.NewElement("init_from");
      snprintf(id, sizeof(id), "image_%u", i);
      initFromXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(id));
      surfaceXml->LinkEndChild(initFromXml);

      newParamXml = this->dataPtr->xmlDoc.NewElement("newparam");
      snprintf(id, sizeof(id), "image_%u_sampler", i);
      newParamXml->SetAttribute("sid", id);
      profileCommonXml->LinkEndChild(newParamXml);

      auto sampler2dXml = this->dataPtr->xmlDoc.NewElement("sampler2D");
      newParamXml->LinkEndChild(sampler2dXml);

      auto sourceXml = this->dataPtr->xmlDoc.NewElement("source");
      snprintf(id, sizeof(id), "image_%u_surface", i);
      sourceXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(id));
      sampler2dXml->LinkEndChild(sourceXml);

      auto minFilterXml = this->dataPtr->xmlDoc.NewElement("minfilter");
      minFilterXml->LinkEndChild(this->dataPtr->xmlDoc.NewText("LINEAR"));
      sampler2dXml->LinkEndChild(minFilterXml);

      auto magFilterXml = this->dataPtr->xmlDoc.NewElement("magfilter");
      magFilterXml->LinkEndChild(this->dataPtr->xmlDoc.NewText("LINEAR"));
      sampler2dXml->LinkEndChild(magFilterXml);
    }

    auto techniqueXml = this->dataPtr->xmlDoc.NewElement("technique");
    techniqueXml->SetAttribute("sid", "COMMON");
    profileCommonXml->LinkEndChild(techniqueXml);

    // gazebo::common::Material::ShadeMode shadeMode =
    //    material->GetShadeMode();

    // Using phong for now
    auto phongXml = this->dataPtr->xmlDoc.NewElement("phong");
    techniqueXml->LinkEndChild(phongXml);

    // ambient
    unsigned int RGBAcolor = material->GetAmbient().GetAsRGBA();
    float r = ((RGBAcolor >> 24) & 0xFF) / 255.0f;
    float g = ((RGBAcolor >> 16) & 0xFF) / 255.0f;
    float b = ((RGBAcolor >> 8) & 0xFF) / 255.0f;
    float a = (RGBAcolor & 0xFF) / 255.0f;

    auto ambientXml = this->dataPtr->xmlDoc.NewElement("ambient");
    phongXml->LinkEndChild(ambientXml);

    auto colorXml = this->dataPtr->xmlDoc.NewElement("color");
    snprintf(id, sizeof(id), "%f %f %f %f", r, g, b, a);
    colorXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(id));
    ambientXml->LinkEndChild(colorXml);

    // emission
    RGBAcolor = material->GetEmissive().GetAsRGBA();
    r = ((RGBAcolor >> 24) & 0xFF) / 255.0f;
    g = ((RGBAcolor >> 16) & 0xFF) / 255.0f;
    b = ((RGBAcolor >> 8) & 0xFF) / 255.0f;
    a = (RGBAcolor & 0xFF) / 255.0f;

    auto emissionXml = this->dataPtr->xmlDoc.NewElement("emission");
    phongXml->LinkEndChild(emissionXml);

    colorXml = this->dataPtr->xmlDoc.NewElement("color");
    snprintf(id, sizeof(id), "%f %f %f %f", r, g, b, a);
    colorXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(id));
    emissionXml->LinkEndChild(colorXml);

    // diffuse
    auto diffuseXml = this->dataPtr->xmlDoc.NewElement("diffuse");
    phongXml->LinkEndChild(diffuseXml);

    if (imageString.find("meshes/") != std::string::npos)
    {
      auto textureXml = this->dataPtr->xmlDoc.NewElement("texture");
      snprintf(id, sizeof(id), "image_%u", i);
      textureXml->SetAttribute("texture", id);
      textureXml->SetAttribute("texcoord", "UVSET0");
      diffuseXml->LinkEndChild(textureXml);
    }
    else
    {
      RGBAcolor = material->GetDiffuse().GetAsRGBA();
      r = ((RGBAcolor >> 24) & 0xFF) / 255.0f;
      g = ((RGBAcolor >> 16) & 0xFF) / 255.0f;
      b = ((RGBAcolor >> 8) & 0xFF) / 255.0f;
      a = (RGBAcolor & 0xFF) / 255.0f;

      colorXml = this->dataPtr->xmlDoc.NewElement("color");
      snprintf(id, sizeof(id), "%f %f %f %f", r, g, b, a);
      colorXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(id));
      diffuseXml->LinkEndChild(colorXml);
    }

    // specular
    RGBAcolor = material->GetSpecular().GetAsRGBA();
    r = ((RGBAcolor >> 24) & 0xFF) / 255.0f;
    g = ((RGBAcolor >> 16) & 0xFF) / 255.0f;
    b = ((RGBAcolor >> 8) & 0xFF) / 255.0f;
    a = (RGBAcolor & 0xFF) / 255.0f;

    auto specularXml = this->dataPtr->xmlDoc.NewElement("specular");
    phongXml->LinkEndChild(specularXml);

    colorXml = this->dataPtr->xmlDoc.NewElement("color");
    snprintf(id, sizeof(id), "%f %f %f %f", r, g, b, a);
    colorXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(id));
    specularXml->LinkEndChild(colorXml);

    // transparency
    double transp = material->GetTransparency();

    auto transparencyXml = this->dataPtr->xmlDoc.NewElement("transparency");
    phongXml->LinkEndChild(transparencyXml);

    auto floatXml = this->dataPtr->xmlDoc.NewElement("float");
    snprintf(id, sizeof(id), "%f", transp);
    floatXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(id));
    transparencyXml->LinkEndChild(floatXml);

    // shininess
    double shine = material->GetShininess();

    auto shininessXml = this->dataPtr->xmlDoc.NewElement("shininess");
    phongXml->LinkEndChild(shininessXml);

    colorXml = this->dataPtr->xmlDoc.NewElement("color");
    snprintf(id, sizeof(id), "%f", shine);
    colorXml->LinkEndChild(this->dataPtr->xmlDoc.NewText(id));
    shininessXml->LinkEndChild(colorXml);
  }
}

//////////////////////////////////////////////////
void ColladaExporter::ExportVisualScenes(
    tinyxml2::XMLElement *_libraryVisualScenesXml)
{
  auto visualSceneXml = this->dataPtr->xmlDoc.NewElement("visual_scene");
  _libraryVisualScenesXml->LinkEndChild(visualSceneXml);
  visualSceneXml->SetAttribute("name", "Scene");
  visualSceneXml->SetAttribute("id", "Scene");

  auto nodeXml = this->dataPtr->xmlDoc.NewElement("node");
  visualSceneXml->LinkEndChild(nodeXml);
  nodeXml->SetAttribute("name", "node");
  nodeXml->SetAttribute("id", "node");

  for (unsigned int i = 0; i < this->dataPtr->subMeshCount; ++i)
  {
    char meshId[100], materialId[100], attributeValue[100];
    snprintf(meshId, sizeof(meshId), "mesh_%u", i);
    snprintf(materialId, sizeof(materialId), "material_%u", i);

    auto instanceGeometryXml =
       this->dataPtr->xmlDoc.NewElement("instance_geometry");
    nodeXml->LinkEndChild(instanceGeometryXml);
    snprintf(attributeValue, sizeof(attributeValue), "#%s", meshId);
    instanceGeometryXml->SetAttribute("url", attributeValue);

    const gazebo::common::Material *material =
        this->dataPtr->mesh->GetMaterial(i);

    if (material)
    {
      auto bindMaterialXml = this->dataPtr->xmlDoc.NewElement("bind_material");
      instanceGeometryXml->LinkEndChild(bindMaterialXml);

      auto techniqueCommonXml =
          this->dataPtr->xmlDoc.NewElement("technique_common");
      bindMaterialXml->LinkEndChild(techniqueCommonXml);

      auto instanceMaterialXml =
          this->dataPtr->xmlDoc.NewElement("instance_material");
      techniqueCommonXml->LinkEndChild(instanceMaterialXml);
      instanceMaterialXml->SetAttribute("symbol", materialId);
      snprintf(attributeValue, sizeof(attributeValue), "#%s", materialId);
      instanceMaterialXml->SetAttribute("target", attributeValue);

      std::string imageString = material->GetTextureImage();

      if (imageString.find("meshes/") != std::string::npos)
      {
        auto bindVertexInputXml =
            this->dataPtr->xmlDoc.NewElement("bind_vertex_input");
        instanceMaterialXml->LinkEndChild(bindVertexInputXml);
        bindVertexInputXml->SetAttribute("semantic", "UVSET0");
        bindVertexInputXml->SetAttribute("input_semantic", "TEXCOORD");
      }
    }
  }
}

//////////////////////////////////////////////////
void ColladaExporter::ExportScene(tinyxml2::XMLElement *_sceneXml)
{
  auto instanceVisualSceneXml =
      this->dataPtr->xmlDoc.NewElement("instance_visual_scene");
  _sceneXml->LinkEndChild(instanceVisualSceneXml);
  instanceVisualSceneXml->SetAttribute("url", "#Scene");
}
