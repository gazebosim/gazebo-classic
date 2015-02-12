/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <gtest/gtest.h>
#include <tinyxml.h>

#include "test_config.h"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/ColladaLoader.hh"
#include "gazebo/common/ColladaExporter.hh"
#include "test/util.hh"

using namespace gazebo;

class ColladaExporter : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(ColladaExporter, ExportBox)
{
  std::string filenameIn = std::string(PROJECT_SOURCE_PATH) +
      "/test/data/box.dae";

  boost::filesystem::path pathOut(boost::filesystem::current_path());
  boost::filesystem::create_directories(pathOut /
      boost::filesystem::path("tmp"));
  std::string filenameOut = pathOut.string() +
      "/tmp/box_exported";

  common::ColladaLoader loader;
  const common::Mesh *meshOriginal = loader.Load(
      filenameIn);

  // Export with extension
  common::ColladaExporter exporter;
  exporter.Export(meshOriginal, filenameOut, false);

  // Check .dae file
  TiXmlDocument xmlDoc;
  filenameOut = filenameOut + ".dae";
  EXPECT_TRUE(xmlDoc.LoadFile(filenameOut));

  const char *countDae = xmlDoc.FirstChildElement("COLLADA")
                               ->FirstChildElement("library_geometries")
                               ->FirstChildElement("geometry")
                               ->FirstChildElement("mesh")
                               ->FirstChildElement("source")
                               ->FirstChildElement("float_array")
                               ->Attribute("count");
  unsigned int countMeshInt = meshOriginal->GetSubMesh(0)->GetVertexCount()*3;
  char countMesh[100];
  snprintf(countMesh, sizeof(countMesh), "%u", countMeshInt);

  EXPECT_STREQ(countDae, countMesh);

  // Reload mesh and compare
  const common::Mesh *meshReloaded = loader.Load(filenameOut);

  EXPECT_EQ(meshOriginal->GetName(), meshReloaded->GetName());
  EXPECT_EQ(meshOriginal->GetMax(), meshReloaded->GetMax());
  EXPECT_EQ(meshOriginal->GetMin(), meshReloaded->GetMin());
  EXPECT_EQ(meshOriginal->GetSubMeshCount(), meshReloaded->GetSubMeshCount());
  EXPECT_EQ(meshOriginal->GetMaterialCount(),
      meshReloaded->GetMaterialCount());
  EXPECT_EQ(meshOriginal->GetIndexCount(), meshReloaded->GetIndexCount());
  EXPECT_EQ(meshOriginal->GetVertexCount(), meshReloaded->GetVertexCount());
  EXPECT_EQ(meshOriginal->GetNormalCount(), meshReloaded->GetNormalCount());
  EXPECT_EQ(meshOriginal->GetTexCoordCount(),
      meshReloaded->GetTexCoordCount());
  for (unsigned int i = 0; i < meshOriginal->GetSubMeshCount(); ++i)
  {
    for (unsigned int j = 0; j < meshOriginal->GetVertexCount(); ++j)
    {
      EXPECT_EQ(meshOriginal->GetSubMesh(i)->GetVertex(j),
          meshReloaded->GetSubMesh(i)->GetVertex(j));
    }
    for (unsigned int j = 0; j < meshOriginal->GetNormalCount(); ++j)
    {
      EXPECT_EQ(meshOriginal->GetSubMesh(i)->GetNormal(j),
          meshReloaded->GetSubMesh(i)->GetNormal(j));
    }
    for (unsigned int j = 0; j < meshOriginal->GetTexCoordCount(); ++j)
    {
      EXPECT_EQ(meshOriginal->GetSubMesh(i)->GetTexCoord(j),
          meshReloaded->GetSubMesh(i)->GetTexCoord(j));
    }
  }

  // Remove temp directory
  boost::filesystem::remove_all(pathOut.string() + "/tmp");
}

/////////////////////////////////////////////////
TEST_F(ColladaExporter, ExportCordlessDrill)
{
  common::ColladaLoader loader;
  const common::Mesh *meshOriginal = loader.Load(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/data/cordless_drill/meshes/cordless_drill.dae");

  boost::filesystem::path pathOut(boost::filesystem::current_path());
  boost::filesystem::create_directories(pathOut /
      boost::filesystem::path("tmp"));

  common::ColladaExporter exporter;
  exporter.Export(meshOriginal, pathOut.string() +
      "/tmp/cordless_drill_exported", true);

  // Check .dae file
  TiXmlDocument xmlDoc;
  EXPECT_TRUE(xmlDoc.LoadFile(pathOut.string() +
      "/tmp/cordless_drill_exported/meshes/cordless_drill_exported.dae"));

  TiXmlElement *geometryXml = xmlDoc.FirstChildElement("COLLADA")
      ->FirstChildElement("library_geometries")
      ->FirstChildElement("geometry");

  for (unsigned int i = 0; i < meshOriginal->GetSubMeshCount(); ++i)
  {
    unsigned int countMeshInt = meshOriginal->GetSubMesh(i)->
        GetVertexCount()*3;
    char countMesh[100];
    snprintf(countMesh, sizeof(countMesh), "%u", countMeshInt);

    const char *countDae = geometryXml
        ->FirstChildElement("mesh")
        ->FirstChildElement("source")
        ->FirstChildElement("float_array")
        ->Attribute("count");

    EXPECT_STREQ(countDae, countMesh);

    geometryXml = geometryXml->NextSiblingElement("geometry");
  }

  // Reload mesh and compare
  const common::Mesh *meshReloaded = loader.Load(pathOut.string() +
      "/tmp/cordless_drill_exported/meshes/cordless_drill_exported.dae");

  EXPECT_EQ(meshOriginal->GetName(), meshReloaded->GetName());
  EXPECT_EQ(meshOriginal->GetMax(), meshReloaded->GetMax());
  EXPECT_EQ(meshOriginal->GetMin(), meshReloaded->GetMin());
  EXPECT_EQ(meshOriginal->GetSubMeshCount(), meshReloaded->GetSubMeshCount());
  EXPECT_EQ(meshOriginal->GetMaterialCount(),
      meshReloaded->GetMaterialCount());
  EXPECT_EQ(meshOriginal->GetIndexCount(), meshReloaded->GetIndexCount());
  EXPECT_EQ(meshOriginal->GetVertexCount(), meshReloaded->GetVertexCount());
  EXPECT_EQ(meshOriginal->GetNormalCount(), meshReloaded->GetNormalCount());
  EXPECT_EQ(meshOriginal->GetTexCoordCount(),
      meshReloaded->GetTexCoordCount());
  for (unsigned int i = 0; i < meshOriginal->GetSubMeshCount(); ++i)
  {
    for (unsigned int j = 0; j < meshOriginal->GetVertexCount(); ++j)
    {
      EXPECT_EQ(meshOriginal->GetSubMesh(i)->GetVertex(j),
          meshReloaded->GetSubMesh(i)->GetVertex(j));
    }
    for (unsigned int j = 0; j < meshOriginal->GetNormalCount(); ++j)
    {
      EXPECT_EQ(meshOriginal->GetSubMesh(i)->GetNormal(j),
          meshReloaded->GetSubMesh(i)->GetNormal(j));
    }
    for (unsigned int j = 0; j < meshOriginal->GetTexCoordCount(); ++j)
    {
      EXPECT_EQ(meshOriginal->GetSubMesh(i)->GetTexCoord(j),
          meshReloaded->GetSubMesh(i)->GetTexCoord(j));
    }
  }

  // Remove temp directory
  boost::filesystem::remove_all(pathOut.string() + "/tmp");
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
