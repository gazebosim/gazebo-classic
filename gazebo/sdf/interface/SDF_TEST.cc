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

#include "gazebo/sdf/sdf.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/gzmath.hh"

class SdfUpdateFixture
{
  public:  std::string GetName() const {return this->name;}
  public:  bool GetFlag() const {return this->flag;}
  public:  gazebo::math::Pose GetPose() const {return this->pose;}
  public:  std::string name;
  public:  bool flag;
  public:  gazebo::math::Pose pose;
};

////////////////////////////////////////////////////
/// Ensure that SDF::Update is working for attributes
TEST(SdfUpdate, UpdateAttribute)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='test_model'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed.root->GetElement("model");

  // Read name attribute value
  EXPECT_TRUE(modelElem->HasAttribute("name"));
  sdf::ParamPtr nameParam = modelElem->GetAttribute("name");
  EXPECT_TRUE(nameParam->IsStr());

  // Set test class variables based on sdf values
  // Set parameter update functions to test class accessors
  SdfUpdateFixture fixture;
  nameParam->Get(fixture.name);
  nameParam->SetUpdateFunc(boost::bind(&SdfUpdateFixture::GetName, &fixture));

  std::string nameCheck;
  int i;
  for (i = 0; i < 4; i++)
  {
    // Update test class variables
    fixture.name[0] = 'd' + i;

    // Update root sdf element
    sdfParsed.root->Update();

    // Expect sdf values to match test class variables
    nameParam->Get(nameCheck);
    EXPECT_EQ(nameCheck, fixture.name);
  }
}

////////////////////////////////////////////////////
/// Ensure that SDF::Update is working for elements
TEST(SdfUpdate, UpdateElement)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='test_model'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("model"));
  sdf::ElementPtr modelElem = sdfParsed.root->GetElement("model");

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
  SdfUpdateFixture fixture;
  staticParam->Get(fixture.flag);
  staticParam->SetUpdateFunc(boost::bind(&SdfUpdateFixture::GetFlag, &fixture));
  poseParam->Get(fixture.pose);
  poseParam->SetUpdateFunc(boost::bind(&SdfUpdateFixture::GetPose, &fixture));

  bool flagCheck;
  gazebo::math::Pose poseCheck;
  int i;
  for (i = 0; i < 4; i++)
  {
    // Update test class variables
    fixture.flag = !fixture.flag;
    fixture.pose.pos.x = i;
    fixture.pose.pos.y = i+10;
    fixture.pose.pos.z = -i*i*i;

    // Update root sdf element
    sdfParsed.root->Update();

    // Expect sdf values to match test class variables
    staticParam->Get(flagCheck);
    EXPECT_EQ(flagCheck, fixture.flag);
    poseParam->Get(poseCheck);
    EXPECT_EQ(poseCheck, fixture.pose);
  }
}

////////////////////////////////////////////////////
/// Ensure that SDF::Element::RemoveFromParent is working
TEST(SdfUpdate, ElementRemoveFromParent)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='model1'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "<model name='model2'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "<model name='model3'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr elem;

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("model"));
  elem = sdfParsed.root->GetElement("model");

  // Select the second model named 'model2'
  elem = elem->GetNextElement("model");
  EXPECT_TRUE(elem);
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->GetValueString("name"), "model2");

  // Remove model2
  elem->RemoveFromParent();

  // Get first model element again
  elem = sdfParsed.root->GetElement("model");
  // Check name == model1
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->GetValueString("name"), "model1");

  // Get next model element
  elem = elem->GetNextElement("model");
  // Check name == model3
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->GetValueString("name"), "model3");

  // Try to get another model element
  elem = elem->GetNextElement("model");
  EXPECT_FALSE(elem);
}

////////////////////////////////////////////////////
/// Ensure that SDF::Element::RemoveChild is working
TEST(SdfUpdate, ElementRemoveChild)
{
  // Set up a simple sdf model file
  std::ostringstream stream;
  stream << "<sdf version='1.3'>"
         << "<model name='model1'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "<model name='model2'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "<model name='model3'>"
         << "  <pose>0 1 2  0 0 0</pose>"
         << "  <static>false</static>"
         << "</model>"
         << "</sdf>";
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr elem, elem2;

  // Verify correct parsing
  EXPECT_TRUE(sdfParsed.root->HasElement("model"));
  elem = sdfParsed.root->GetElement("model");

  // Select the static element in model1
  elem2 = elem->GetElement("static");
  EXPECT_TRUE(elem2);
  EXPECT_FALSE(elem2->GetValueBool());
  elem->RemoveChild(elem2);

  // Get first model element again
  elem = sdfParsed.root->GetElement("model");
  // Check name == model1
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->GetValueString("name"), "model1");

  // Check that we have deleted the static element in model1
  EXPECT_FALSE(elem->HasElement("static"));

  // Get model2
  elem2 = elem->GetNextElement("model");

  // Remove model2
  sdfParsed.root->RemoveChild(elem2);

  // Get first model element again
  elem = sdfParsed.root->GetElement("model");
  // Check name == model1
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->GetValueString("name"), "model1");

  // Get next model element
  elem = elem->GetNextElement("model");
  // Check name == model3
  EXPECT_TRUE(elem->HasAttribute("name"));
  EXPECT_EQ(elem->GetValueString("name"), "model3");

  // Try to get another model element
  elem = elem->GetNextElement("model");
  EXPECT_FALSE(elem);
}

////////////////////////////////////////////////////
/// Ensure that getting empty values with empty keys returns correct values.
TEST(SdfUpdate, EmptyValues)
{
  std::string emptyString;
  sdf::ElementPtr elem;

  elem.reset(new sdf::Element());
  EXPECT_FALSE(elem->GetValueBool(emptyString));
  elem->AddValue("bool", "true", "0", "description");
  EXPECT_TRUE(elem->GetValueBool(emptyString));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueInt(emptyString), 0);
  elem->AddValue("int", "12", "0", "description");
  EXPECT_EQ(elem->GetValueInt(emptyString), 12);

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueUInt(emptyString), static_cast<unsigned int>(0));
  elem->AddValue("unsigned int", "123", "0", "description");
  EXPECT_EQ(elem->GetValueUInt(emptyString), static_cast<unsigned int>(123));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueChar(emptyString), '\0');
  elem->AddValue("char", "a", "0", "description");
  EXPECT_EQ(elem->GetValueChar(emptyString), 'a');

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueString(emptyString), "");
  elem->AddValue("string", "hello", "0", "description");
  EXPECT_EQ(elem->GetValueString(emptyString), "hello");

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueVector2d(emptyString), gazebo::math::Vector2d());
  elem->AddValue("vector2d", "1 2", "0", "description");
  EXPECT_EQ(elem->GetValueVector2d(emptyString), gazebo::math::Vector2d(1, 2));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueVector3(emptyString), gazebo::math::Vector3());
  elem->AddValue("vector3", "1 2 3", "0", "description");
  EXPECT_EQ(elem->GetValueVector3(emptyString), gazebo::math::Vector3(1, 2, 3));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueQuaternion(emptyString), gazebo::math::Quaternion());
  elem->AddValue("quaternion", "1 2 3", "0", "description");
  EXPECT_EQ(elem->GetValueQuaternion(emptyString),
            gazebo::math::Quaternion(-2.14159, 1.14159, -0.141593));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValuePose(emptyString), gazebo::math::Pose());
  elem->AddValue("pose", "1 2 3 4 5 6", "0", "description");
  EXPECT_EQ(elem->GetValuePose(emptyString),
            gazebo::math::Pose(1, 2, 3, 4, 5, 6));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueColor(emptyString), gazebo::common::Color());
  elem->AddValue("color", ".1 .2 .3 1.0", "0", "description");
  EXPECT_EQ(elem->GetValueColor(emptyString),
            gazebo::common::Color(.1, .2, .3, 1.0));

  elem.reset(new sdf::Element());
  EXPECT_EQ(elem->GetValueTime(emptyString), gazebo::common::Time());
  elem->AddValue("time", "1 2", "0", "description");
  EXPECT_EQ(elem->GetValueTime(emptyString), gazebo::common::Time(1, 2));

  elem.reset(new sdf::Element());
  EXPECT_NEAR(elem->GetValueFloat(emptyString), 0.0, 1e-6);
  elem->AddValue("float", "12.34", "0", "description");
  EXPECT_NEAR(elem->GetValueFloat(emptyString), 12.34, 1e-6);

  elem.reset(new sdf::Element());
  EXPECT_NEAR(elem->GetValueDouble(emptyString), 0.0, 1e-6);
  elem->AddValue("double", "12.34", "0", "description");
  EXPECT_NEAR(elem->GetValueDouble(emptyString), 12.34, 1e-6);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
