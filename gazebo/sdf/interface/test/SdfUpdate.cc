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
#include "gazebo/math/Pose.hh"

class SdfUpdateElement
{
  public:  std::string GetName() const {return this->name;}
  public:  bool GetFlag() const {return this->flag;}
  public:  gazebo::math::Pose GetPose() const {return this->pose;}
  public:  std::string name;
  public:  bool flag;
  public:  gazebo::math::Pose pose;
};

/////////////////////////////////////////////////
/// Ensure that SDF::Update is working
TEST(SdfUpdate, UpdateElement)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='test_model'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</true>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdf_parsed;
  sdf_parsed.SetFromString(stream.str());

  // Verify correct parsing
  EXPECT_TRUE(sdf_parsed.root->HasElement("model"));
  sdf::ElementPtr modelElem = sdf_parsed.root->GetElement("model");

  // Read name attribute value
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  sdf::ParamPtr nameParam = modelElem->GetAttribute("name");
  EXPECT_TRUE(nameParam->IsStr());

  // Read static element value
  EXPECT_TRUE(modelElem->HasElement("static"));
  sdf::ParamPtr staticParam = modelElem->GetElement("static")->GetValue();
  EXPECT_TRUE(staticParam->IsBool());

  // Read pose element value
  EXPECT_TRUE(modelElem->HasElement("pose"));
  sdf::ParamPtr poseParam = modelElem->GetElement("pose")->GetValue();
  EXPECT_TRUE(poseParam->IsPose());

  // Set test class variables based on sdf values
  // Set parameter update functions to test class accessors
  SdfUpdateElement testClass;
  nameParam->Get(testClass.name);
  nameParam->SetUpdateFunc(boost::bind(&SdfUpdateElement::GetName, &testClass));
  staticParam->Get(testClass.flag);
  staticParam->SetUpdateFunc(boost::bind(&SdfUpdateElement::GetFlag, &testClass));
  poseParam->Get(testClass.pose);
  poseParam->SetUpdateFunc(boost::bind(&SdfUpdateElement::GetPose, &testClass));

  std::string nameCheck;
  bool flagCheck;
  gazebo::math::Pose poseCheck;
  int i;
  for (i=0; i < 4; i++)
  {
    // Update test class variables
    testClass.name[0] = 'd' + i;
    testClass.flag = !testClass.flag;
    testClass.pose.pos.x = i;
    testClass.pose.pos.y = i+10;
    testClass.pose.pos.z = -i*i*i;

    // Update root sdf element
    sdf_parsed.root->Update();

    // Expect sdf values to match test class variables
    nameParam->Get(nameCheck);
    EXPECT_EQ(nameCheck, testClass.name);
    staticParam->Get(flagCheck);
    EXPECT_EQ(flagCheck, testClass.flag);
    poseParam->Get(poseCheck);
    EXPECT_EQ(poseCheck, testClass.pose);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
