/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
  std::string filename = std::string(PROJECT_SOURCE_PATH) + "/test/data/box";

  common::ColladaLoader loader;
  const common::Mesh *meshOriginal = loader.Load(
      filename + ".dae");

  // Export with extension
  common::ColladaExporter exporter;
  exporter.Export(meshOriginal, filename + "_exported.dae");

  // Check .dae file
  TiXmlDocument xmlDoc;
  EXPECT_TRUE(xmlDoc.LoadFile(filename + "_exported.dae"));

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
  const common::Mesh *meshReloaded = loader.Load(filename + "_exported.dae");

  EXPECT_EQ(meshOriginal->GetName(), meshReloaded->GetName());
  EXPECT_EQ(meshOriginal->GetMax(), meshReloaded->GetMax());
  EXPECT_EQ(meshOriginal->GetMin(), meshReloaded->GetMin());
  EXPECT_EQ(meshOriginal->GetSubMeshCount(), meshReloaded->GetSubMeshCount());
  EXPECT_EQ(meshOriginal->GetMaterialCount(),
      meshReloaded->GetMaterialCount());
  EXPECT_EQ(meshOriginal->GetVertexCount(), meshReloaded->GetVertexCount());
  EXPECT_EQ(meshOriginal->GetNormalCount(), meshReloaded->GetNormalCount());
  EXPECT_EQ(meshOriginal->GetTexCoordCount(),
      meshReloaded->GetTexCoordCount());
  EXPECT_EQ(meshOriginal->GetIndexCount(), meshReloaded->GetIndexCount());
}

/////////////////////////////////////////////////
TEST_F(ColladaExporter, ExportCordlessDrill)
{
  std::string filename = std::string(PROJECT_SOURCE_PATH) +
      "/test/data/cordless_drill/meshes/cordless_drill";

  common::ColladaLoader loader;
  const common::Mesh *meshOriginal = loader.Load(
      filename + ".dae");

  // Export without extension
  common::ColladaExporter exporter;
  exporter.Export(meshOriginal, filename + "_exported");

  // Check .dae file
  TiXmlDocument xmlDoc;
  EXPECT_TRUE(xmlDoc.LoadFile(filename + "_exported.dae"));

  TiXmlElement *geometryXml = xmlDoc.FirstChildElement("COLLADA")
      ->FirstChildElement("library_geometries")
      ->FirstChildElement("geometry");

  for (unsigned int i = 0; i < meshOriginal->GetSubMeshCount(); i++)
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
  const common::Mesh *meshReloaded = loader.Load(filename + "_exported.dae");

  EXPECT_EQ(meshOriginal->GetName(), meshReloaded->GetName());
  EXPECT_EQ(meshOriginal->GetMax(), meshReloaded->GetMax());
  EXPECT_EQ(meshOriginal->GetMin(), meshReloaded->GetMin());
  EXPECT_EQ(meshOriginal->GetSubMeshCount(), meshReloaded->GetSubMeshCount());
  EXPECT_EQ(meshOriginal->GetMaterialCount(),
      meshReloaded->GetMaterialCount());
  EXPECT_EQ(meshOriginal->GetVertexCount(), meshReloaded->GetVertexCount());
  EXPECT_EQ(meshOriginal->GetNormalCount(), meshReloaded->GetNormalCount());
  EXPECT_EQ(meshOriginal->GetTexCoordCount(),
      meshReloaded->GetTexCoordCount());
  EXPECT_EQ(meshOriginal->GetIndexCount(), meshReloaded->GetIndexCount());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
