/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include "gazebo/sdf/sdf.hh"
#include "gazebo/sdf/interface/parser_urdf.hh"

#include "test_config.h"


const std::string URDF_TEST_FILE = std::string(PROJECT_SOURCE_PATH) + "/test/performance/parser_urdf_atlas.urdf";

TEST(URDFParser, AtlasURDF_5runs_performance)
{
   urdf2gazebo::URDF2Gazebo parser;
   for (int i = 0; i < 5; i++)
       TiXmlDocument sdf_result = parser.InitModelFile(URDF_TEST_FILE);
}
