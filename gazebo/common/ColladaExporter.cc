/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <tinyxml.h>
//#include <math.h>
//#include <sstream>
//#include <boost/algorithm/string.hpp>
//#include <boost/lexical_cast.hpp>

//#include "gazebo/math/Helpers.hh"
//#include "gazebo/math/Angle.hh"
//#include "gazebo/math/Vector2d.hh"
//#include "gazebo/math/Vector3.hh"
//#include "gazebo/math/Matrix4.hh"
//#include "gazebo/math/Quaternion.hh"
//#include "gazebo/common/Console.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/Mesh.hh"
//#include "gazebo/common/Skeleton.hh"
//#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/common/ColladaExporter.hh"
//#include "gazebo/common/SystemPaths.hh"
//#include "gazebo/common/Exception.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
  ColladaExporter::ColladaExporter()
: MeshExporter(), meter(1.0)
{
}

//////////////////////////////////////////////////
ColladaExporter::~ColladaExporter()
{
}

////////////////////////////////////////////////////
void ColladaExporter::Export(const Mesh *_mesh)
{
  this->mesh = _mesh;
  this->materialCount = this->mesh->GetMaterialCount();

  // Mesh name
  std::string meshName = _mesh->GetName();
  meshName = meshName.substr(0,meshName.find(".dae"));

  // Collada file
  TiXmlDocument xmlDoc;

  // XML declaration
  TiXmlDeclaration* declaration = new TiXmlDeclaration( "1.0", "utf-8", "" );
  xmlDoc.LinkEndChild( declaration );

  // Collada element
  TiXmlElement *colladaXml = new TiXmlElement( "COLLADA" );
  xmlDoc.LinkEndChild( colladaXml );
  colladaXml->SetAttribute("version", "1.4.1");
  colladaXml->SetAttribute("xmlns",
      "http://www.collada.org/2005/11/COLLADASchema");

  // Asset element
  TiXmlElement *assetXml = new TiXmlElement( "asset" );
  this->ExportAsset(assetXml);
  colladaXml->LinkEndChild( assetXml );

  // Library geometries element
  TiXmlElement *library_geometriesXml = new TiXmlElement( "library_geometries" );
  this->ExportGeometries(library_geometriesXml);
  colladaXml->LinkEndChild( library_geometriesXml );

  if(this->materialCount != 0)
  {
    // Library images element
    TiXmlElement *library_imagesXml = new TiXmlElement( "library_images" );
    int imageCount = this->ExportImages(library_imagesXml);
    if(imageCount)
    {
      colladaXml->LinkEndChild( library_imagesXml );
    }

    // Library materials element
    TiXmlElement *library_materialsXml = new TiXmlElement( "library_materials" );
    this->ExportMaterials(library_materialsXml);
    colladaXml->LinkEndChild( library_materialsXml );

    // Library effects element
    TiXmlElement *library_effectsXml = new TiXmlElement( "library_effects" );
    this->ExportEffects(library_effectsXml);
    colladaXml->LinkEndChild( library_effectsXml );
  }

  // Library visual scenes element
  TiXmlElement *library_visual_scenesXml =
      new TiXmlElement( "library_visual_scenes" );
  this->ExportVisualScenes(library_visual_scenesXml);
  colladaXml->LinkEndChild( library_visual_scenesXml );

  // Scene element
  TiXmlElement *sceneXml = new TiXmlElement( "scene" );
  this->ExportScene(sceneXml);
  colladaXml->LinkEndChild( sceneXml );

  xmlDoc.SaveFile( meshName+"_exported.dae" );
}

///////////////////////////////////////////////////
void ColladaExporter::ExportAsset(TiXmlElement *_assetXml)
{
  // unit
  TiXmlElement * unitXml = new TiXmlElement( "unit" );
  unitXml->SetAttribute("meter", "1");
  unitXml->SetAttribute("name", "meter");
  _assetXml->LinkEndChild( unitXml );

  // up_axis
  TiXmlElement * up_axisXml = new TiXmlElement( "up_axis" );
  up_axisXml->LinkEndChild( new TiXmlText( "Z_UP" ));
  _assetXml->LinkEndChild( up_axisXml );
}

//////////////////////////////////////////////////
// TODO: clean
void ColladaExporter::FillGeometrySources(
    const gazebo::common::SubMesh *_subMesh,
    TiXmlElement *Mesh, int type, const char *meshID)
{

  std::ostringstream sourceID;
  std::ostringstream sourceArrayID;
  std::ostringstream sourceArrayIdSelector;
  std::ostringstream fillData;
  fillData.precision(5);
  fillData<<std::fixed;
  int stride;
  unsigned int count = 0;

  if (type == 1)
  {
    sourceID << meshID << "-Positions";
    count = _subMesh->GetVertexCount();
    stride = 3;
    gazebo::math::Vector3 vertex;
    for(unsigned int i = 0; i < count; ++i)
    {
      vertex = _subMesh->GetVertex(i);
      fillData << vertex.x << " " << vertex.y << " " << vertex.z << " ";
    }
  }
  if (type == 2)
  {
    sourceID << meshID << "-Normals";
    count = _subMesh->GetNormalCount();
    stride = 3;
    gazebo::math::Vector3 normal;
    for(unsigned int i = 0; i < count; ++i)
    {
      normal = _subMesh->GetNormal(i);
      fillData << normal.x << " " << normal.y << " " << normal.z << " ";
    }
  }
  sourceArrayID << sourceID.str() << "-array";
  sourceArrayIdSelector << "#" << sourceArrayID.str();

  TiXmlElement *source = new TiXmlElement( "source" );
  Mesh->LinkEndChild( source );
  source->SetAttribute("id", sourceID.str().c_str());
  source->SetAttribute("name", sourceID.str().c_str());

  TiXmlElement *float_array = new TiXmlElement( "float_array" );
  float_array->SetAttribute("count", count*stride);
  float_array->SetAttribute("id", sourceArrayID.str().c_str());
  float_array->LinkEndChild( new TiXmlText( fillData.str().c_str() ));
  source->LinkEndChild( float_array );

  TiXmlElement *technique_common = new TiXmlElement( "technique_common" );
  source->LinkEndChild( technique_common );

  TiXmlElement *accessor = new TiXmlElement( "accessor" );
  accessor->SetAttribute("count", count);
  accessor->SetAttribute("source", sourceArrayIdSelector.str().c_str());
  accessor->SetAttribute("stride", stride);
  technique_common->LinkEndChild( accessor );

  TiXmlElement *param = new TiXmlElement( "param" );

  if (type == 1 || type == 2)
  {
    param->SetAttribute("type", "float");
    param->SetAttribute("name", "X");
    accessor->LinkEndChild( param );

    param = new TiXmlElement( "param" );
    param->SetAttribute("type", "float");
    param->SetAttribute("name", "Y");
    accessor->LinkEndChild( param );

    param = new TiXmlElement( "param" );
    param->SetAttribute("type", "float");
    param->SetAttribute("name", "Z");
    accessor->LinkEndChild( param );
  }
}

//////////////////////////////////////////////////
// TODO: clean
void ColladaExporter::FillTextureSource(
    const gazebo::common::SubMesh *_subMesh,
    TiXmlElement *Mesh,
    const char *meshID)
{
  // For collada
  std::ostringstream sourceID;
  std::ostringstream sourceArrayID;
  std::ostringstream sourceArrayIdSelector;
  unsigned int outCount = _subMesh->GetVertexCount();
  int stride = 2;
  std::ostringstream fillData;
  fillData.precision(5);
  fillData<<std::fixed;

  gazebo::math::Vector2d inTexCoord;
  for(long unsigned int i = 0; i < outCount; ++i)
  {
    inTexCoord = _subMesh->GetTexCoord(i);
    fillData << inTexCoord.x << " " << 1-inTexCoord.y << " ";
  }

  sourceID << meshID << "-UVMap";
  sourceArrayID << sourceID.str() << "-array";
  sourceArrayIdSelector << "#" << sourceArrayID.str();

  TiXmlElement *source = new TiXmlElement( "source" );
  Mesh->LinkEndChild( source );
  source->SetAttribute("id", sourceID.str().c_str());
  source->SetAttribute("name", sourceID.str().c_str());

  TiXmlElement *float_array = new TiXmlElement( "float_array" );
  float_array->SetAttribute("count", outCount*stride);
  float_array->SetAttribute("id", sourceArrayID.str().c_str());
  float_array->LinkEndChild( new TiXmlText( fillData.str().c_str() ));
  source->LinkEndChild( float_array );

  TiXmlElement *technique_common = new TiXmlElement( "technique_common" );
  source->LinkEndChild( technique_common );

  TiXmlElement *accessor = new TiXmlElement( "accessor" );
  accessor->SetAttribute("count", outCount);
  accessor->SetAttribute("source", sourceArrayIdSelector.str().c_str());
  accessor->SetAttribute("stride", stride);
  technique_common->LinkEndChild( accessor );

  TiXmlElement *param = new TiXmlElement( "param" );

  param->SetAttribute("type", "float");
  param->SetAttribute("name", "U");
  accessor->LinkEndChild( param );

  param = new TiXmlElement( "param" );
  param->SetAttribute("type", "float");
  param->SetAttribute("name", "V");
  accessor->LinkEndChild( param );
}

///////////////////////////////////////////////////
void ColladaExporter::ExportGeometries(TiXmlElement *_library_geometriesXml)
{
  unsigned int meshCount = this->mesh->GetSubMeshCount();

  for(unsigned int i = 0; i < meshCount; i++)
  {
    char meshId[100], materialId[100];
    sprintf(meshId,"mesh_%d", i);
    sprintf(materialId,"material_%d", i);

    TiXmlElement *geometry = new TiXmlElement( "geometry" );
    geometry->SetAttribute("id", meshId);
    _library_geometriesXml->LinkEndChild( geometry );

    TiXmlElement *Mesh = new TiXmlElement( "mesh" );
    geometry->LinkEndChild( Mesh );

    const gazebo::common::SubMesh *subMesh = this->mesh->GetSubMesh(i);

    // Position
    FillGeometrySources(subMesh, Mesh, 1, meshId);
    // Normals
    FillGeometrySources(subMesh, Mesh, 2, meshId);
    if (subMesh->GetTexCoordCount() != 0)
    {
      FillTextureSource(subMesh, Mesh, meshId);
    }

    // Vertices
    char attributeValue[100];

    TiXmlElement *vertices = new TiXmlElement( "vertices" );
    Mesh->LinkEndChild( vertices );
    strcpy(attributeValue,meshId);
    strcat(attributeValue,"-Vertex");
    vertices->SetAttribute("id", attributeValue);
    vertices->SetAttribute("name", attributeValue);

    TiXmlElement *input = new TiXmlElement( "input" );
    vertices->LinkEndChild( input );
    input->SetAttribute("semantic", "POSITION");
    strcpy(attributeValue,"#");
    strcat(attributeValue,meshId);
    strcat(attributeValue,"-Positions");
    input->SetAttribute("source", attributeValue);

    // Triangles
    unsigned int indexCount = subMesh->GetIndexCount();

    TiXmlElement *triangles = new TiXmlElement( "triangles" );
    Mesh->LinkEndChild( triangles );
    triangles->SetAttribute("count", indexCount/3);
    triangles->SetAttribute("material", materialId);

    input = new TiXmlElement( "input" );
    triangles->LinkEndChild( input );
    input->SetAttribute("offset", 0);
    input->SetAttribute("semantic", "VERTEX");
    strcpy(attributeValue,"#");
    strcat(attributeValue,meshId);
    strcat(attributeValue,"-Vertex");
    input->SetAttribute("source", attributeValue);

    input = new TiXmlElement( "input" );
    triangles->LinkEndChild( input );
    input->SetAttribute("offset", 1);
    input->SetAttribute("semantic", "NORMAL");
    strcpy(attributeValue,"#");
    strcat(attributeValue,meshId);
    strcat(attributeValue,"-Normals");
    input->SetAttribute("source", attributeValue);

    if (subMesh->GetTexCoordCount() != 0)
    {
      input = new TiXmlElement( "input" );
      triangles->LinkEndChild( input );
      input->SetAttribute("offset", 2);
      input->SetAttribute("semantic", "TEXCOORD");
      strcpy(attributeValue,"#");
      strcat(attributeValue,meshId);
      strcat(attributeValue,"-UVMap");
      input->SetAttribute("source", attributeValue);
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

    TiXmlElement *p = new TiXmlElement( "p" );
    triangles->LinkEndChild( p );
    p->LinkEndChild( new TiXmlText( fillData.str().c_str() ));
  }
}

///////////////////////////////////////////////////
int ColladaExporter::ExportImages(TiXmlElement *_library_imagesXml)
{
  int imageCount = 0;
  for(unsigned int i = 0; i < this->materialCount; i++)
  {
    const gazebo::common::Material * material = this->mesh->GetMaterial(i);
    std::string imageString = material->GetTextureImage();

    if (imageString.find("meshes/") != std::string::npos)
    {
      char id[100];
      sprintf(id,"image_%d", i);

      TiXmlElement *image = new TiXmlElement( "image" );
      image->SetAttribute("id", id);
      _library_imagesXml->LinkEndChild( image );

      TiXmlElement *init_from = new TiXmlElement( "init_from" );
      init_from->LinkEndChild( new TiXmlText(
        imageString.substr(imageString.find("meshes/")+7) ));
      image->LinkEndChild( init_from );

      imageCount++;
    }
  }

  return imageCount;
}

//////////////////////////////////////////////////
void ColladaExporter::ExportMaterials(TiXmlElement *_library_materialsXml)
{
  for(unsigned int i = 0; i < this->materialCount; i++)
  {
    char id[100];
    sprintf(id,"material_%d", i);

    TiXmlElement *material = new TiXmlElement( "material" );
    material->SetAttribute("id", id);
    _library_materialsXml->LinkEndChild( material );

    sprintf(id,"#material_%d_fx", i);
    TiXmlElement *instance_effect = new TiXmlElement( "instance_effect" );
    instance_effect->SetAttribute("url",id);
    material->LinkEndChild( instance_effect );
  }
}

//////////////////////////////////////////////////
void ColladaExporter::ExportEffects(TiXmlElement *_library_effectsXml)
{
  for(unsigned int i = 0; i < this->materialCount; i++)
  {
    char id[100];
    sprintf(id,"material_%d_fx", i);

    TiXmlElement *effect = new TiXmlElement( "effect" );
    effect->SetAttribute("id", id);
    _library_effectsXml->LinkEndChild( effect );

    TiXmlElement *profile_COMMON = new TiXmlElement( "profile_COMMON" );
    effect->LinkEndChild( profile_COMMON );

    // Image
    const gazebo::common::Material * material = this->mesh->GetMaterial(i);
    std::string imageString = material->GetTextureImage();

    if (imageString.find("meshes/") != std::string::npos)
    {
      TiXmlElement *newparam = new TiXmlElement( "newparam" );
      sprintf(id,"image_%d_surface", i);
      newparam->SetAttribute("sid", id);
      profile_COMMON->LinkEndChild( newparam );

      TiXmlElement *surface = new TiXmlElement( "surface" );
      surface->SetAttribute("type", "2D");
      newparam->LinkEndChild( surface );

      TiXmlElement *init_from = new TiXmlElement( "init_from" );
      sprintf(id,"image_%d", i);
      init_from->LinkEndChild( new TiXmlText(id) );
      surface->LinkEndChild( init_from );

      newparam = new TiXmlElement( "newparam" );
      sprintf(id,"image_%d_sampler", i);
      newparam->SetAttribute("sid", id);
      profile_COMMON->LinkEndChild( newparam );

      TiXmlElement *sampler2D = new TiXmlElement( "sampler2D" );
      newparam->LinkEndChild( sampler2D );

      TiXmlElement *source = new TiXmlElement( "source" );
      sprintf(id,"image_%d_surface", i);
      source->LinkEndChild( new TiXmlText(id) );
      sampler2D->LinkEndChild( source );

      TiXmlElement *minfilter = new TiXmlElement( "minfilter" );
      minfilter->LinkEndChild( new TiXmlText("LINEAR") );
      sampler2D->LinkEndChild( minfilter );

      TiXmlElement *magfilter = new TiXmlElement( "magfilter" );
      magfilter->LinkEndChild( new TiXmlText("LINEAR") );
      sampler2D->LinkEndChild( magfilter );
    }

    TiXmlElement *technique = new TiXmlElement( "technique" );
    technique->SetAttribute("sid", "COMMON");
    profile_COMMON->LinkEndChild( technique );

    //gazebo::common::Material::ShadeMode shadeMode = material->GetShadeMode();

    // Using blinn for now
    TiXmlElement *blinn = new TiXmlElement( "blinn" );
    technique->LinkEndChild( blinn );

    // ambient
    unsigned int RGBAcolor = material->GetAmbient().GetAsRGBA();
    float r = ((RGBAcolor >> 24) & 0xFF) / 255.0f;
    float g = ((RGBAcolor >> 16) & 0xFF) / 255.0f;
    float b = ((RGBAcolor >> 8) & 0xFF) / 255.0f;
    float a = (RGBAcolor & 0xFF) / 255.0f;

    TiXmlElement *ambient = new TiXmlElement( "ambient" );
    blinn->LinkEndChild( ambient );

    TiXmlElement *color = new TiXmlElement( "color" );
    sprintf(id,"%f %f %f %f", r,g,b,a);
    color->LinkEndChild( new TiXmlText( id ));
    ambient->LinkEndChild( color );

    // emission
    RGBAcolor = material->GetEmissive().GetAsRGBA();
    r = ((RGBAcolor >> 24) & 0xFF) / 255.0f;
    g = ((RGBAcolor >> 16) & 0xFF) / 255.0f;
    b = ((RGBAcolor >> 8) & 0xFF) / 255.0f;
    a = (RGBAcolor & 0xFF) / 255.0f;

    TiXmlElement *emission = new TiXmlElement( "emission" );
    blinn->LinkEndChild( emission );

    color = new TiXmlElement( "color" );
    sprintf(id,"%f %f %f %f", r,g,b,a);
    color->LinkEndChild( new TiXmlText( id ));
    emission->LinkEndChild( color );

    // diffuse
    TiXmlElement *diffuse = new TiXmlElement( "diffuse" );
    blinn->LinkEndChild( diffuse );

    if (imageString.find("meshes/") != std::string::npos)
    {
      TiXmlElement *texture = new TiXmlElement( "texture" );
      sprintf(id,"image_%d", i);
      texture->SetAttribute("texture",id);
      texture->SetAttribute("texcoord","UVSET0");
      diffuse->LinkEndChild( texture );
    }
    else
    {
      RGBAcolor = material->GetDiffuse().GetAsRGBA();
      r = ((RGBAcolor >> 24) & 0xFF) / 255.0f;
      g = ((RGBAcolor >> 16) & 0xFF) / 255.0f;
      b = ((RGBAcolor >> 8) & 0xFF) / 255.0f;
      a = (RGBAcolor & 0xFF) / 255.0f;

      color = new TiXmlElement( "color" );
      sprintf(id,"%f %f %f %f", r,g,b,a);
      color->LinkEndChild( new TiXmlText( id ));
      diffuse->LinkEndChild( color );
    }

    // specular
    RGBAcolor = material->GetSpecular().GetAsRGBA();
    r = ((RGBAcolor >> 24) & 0xFF) / 255.0f;
    g = ((RGBAcolor >> 16) & 0xFF) / 255.0f;
    b = ((RGBAcolor >> 8) & 0xFF) / 255.0f;
    a = (RGBAcolor & 0xFF) / 255.0f;

    TiXmlElement *specular = new TiXmlElement( "specular" );
    blinn->LinkEndChild( specular );

    color = new TiXmlElement( "color" );
    sprintf(id,"%f %f %f %f", r,g,b,a);
    color->LinkEndChild( new TiXmlText( id ));
    specular->LinkEndChild( color );

    // transparency
    double transp = material->GetTransparency();

    TiXmlElement *transparency = new TiXmlElement( "transparency" );
    blinn->LinkEndChild( transparency );

    TiXmlElement * Float = new TiXmlElement( "float" );
    sprintf(id,"%f", transp);
    Float->LinkEndChild( new TiXmlText( id ));
    transparency->LinkEndChild( Float );

    // shininess
    double shine = material->GetShininess();

    TiXmlElement *shininess = new TiXmlElement( "shininess" );
    blinn->LinkEndChild( shininess );

    color = new TiXmlElement( "color" );
    sprintf(id,"%f", shine);
    color->LinkEndChild( new TiXmlText( id ));
    shininess->LinkEndChild( color );

  }
}

//////////////////////////////////////////////////
void ColladaExporter::ExportVisualScenes(TiXmlElement *_library_visual_scenesXml)
{
  TiXmlElement *visual_scene = new TiXmlElement( "visual_scene" );
  _library_visual_scenesXml->LinkEndChild( visual_scene );
  visual_scene->SetAttribute("name", "Scene");
  visual_scene->SetAttribute("id", "Scene");

  TiXmlElement *node = new TiXmlElement( "node" );
  visual_scene->LinkEndChild( node );
  node->SetAttribute("name", "node");
  node->SetAttribute("id", "node");

  for(unsigned int i = 0; i < this->mesh->GetSubMeshCount(); i++)
  {
    char meshId[100], materialId[100], attributeValue[100];
    sprintf(meshId,"mesh_%d", i);
    sprintf(materialId,"material_%d", i);

    TiXmlElement *instance_geometry = new TiXmlElement( "instance_geometry" );
    node->LinkEndChild( instance_geometry );
    strcpy(attributeValue,"#");
    strcat(attributeValue,meshId);
    instance_geometry->SetAttribute("url", attributeValue);

    const gazebo::common::Material * material = this->mesh->GetMaterial(i);

    if(material)
    {
      TiXmlElement *bind_material = new TiXmlElement( "bind_material" );
      instance_geometry->LinkEndChild( bind_material );

      TiXmlElement *techniqueCommon = new TiXmlElement( "technique_common" );
      bind_material->LinkEndChild( techniqueCommon );

      TiXmlElement *instanceMaterial = new TiXmlElement( "instance_material" );
      techniqueCommon->LinkEndChild( instanceMaterial );
      instanceMaterial->SetAttribute("symbol", materialId);
      strcpy(attributeValue,"#");
      strcat(attributeValue,materialId);
      instanceMaterial->SetAttribute("target", attributeValue);

      std::string imageString = material->GetTextureImage();

      if (imageString.find("meshes/") != std::string::npos)
      {
        TiXmlElement *bindVertexInput = new TiXmlElement( "bind_vertex_input" );
        instanceMaterial->LinkEndChild( bindVertexInput );
        bindVertexInput->SetAttribute("semantic", "UVSET0");
        bindVertexInput->SetAttribute("input_semantic", "TEXCOORD");
      }
    }
  }
}

//////////////////////////////////////////////////
void ColladaExporter::ExportScene(TiXmlElement *_sceneXml)
{
  TiXmlElement *instance_visual_scene =
      new TiXmlElement( "instance_visual_scene" );
  _sceneXml->LinkEndChild( instance_visual_scene );
  instance_visual_scene->SetAttribute("url", "#Scene");
}
