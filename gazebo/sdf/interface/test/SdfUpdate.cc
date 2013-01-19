/*
 * Copyright 2012 Open Source Robotics Foundation
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

class SdfUpdateElement
{
  public:  bool GetStatic() const;
  private: bool flag;
};

/////////////////////////////////////////////////
/// Ensure that SDF::Update is working
TEST(SdfUpdate, UpdateElement)
{
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='test_model'>"
         << "  <static>false</true>"
         << "</model>"
         << "</sdf>";
  
  sdf::SDF sdf_parsed;
  sdf_parsed.SetFromString(stream.str());

  EXPECT_TRUE(sdf_parsed.root->HasElement("model"));

  ElementPtr modelElem = sdf_parsed.root->GetElement("model");

  EXPECT_TRUE(modelElem->HasElement("static"));

  SdfUpdateElement testClass;
  ParamPtr staticParam = modelElem->GetElement("static")->GetValue();
  EXPECT_TRUE(staticParam->IsBool());
  staticParam->Set(testClass.flag);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
