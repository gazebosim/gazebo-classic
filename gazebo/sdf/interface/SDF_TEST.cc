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
/// Ensure that Converter::Move function is working
TEST(Converter, Move)
{
  // Set up a simple xml file
  std::stringstream stream;
  stream << "<elemA attrA='A'>"
         << "  <elemB attrB='B'>"
         << "    <elemC attrC='C'>"
         << "      <elemD>D</elemD>"
         << "    </elemC>"
         << "  </elemB>"
         << "</elemA>";

  // Verify the xml
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(stream.str().c_str());
  TiXmlElement *childElem =  xmlDoc.FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemA");
  childElem =  childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemB");
  childElem =  childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemC");
  childElem =  childElem->FirstChildElement();
  EXPECT_TRUE(childElem);
  EXPECT_EQ(childElem->ValueStr(), "elemD");

  // Test moving from elem to elem
  // Set up a convert file
  std::stringstream convertStream;
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "     <from element='elemC::elemD'/>"
                << "     <to element='elemE'/>"
                << "   </move>"
                << " </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc;
  convertXmlDoc.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc, &convertXmlDoc);

  TiXmlElement *convertedElem =  xmlDoc.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemC"));
  EXPECT_TRUE(convertedElem->FirstChildElement("elemE"));
  std::string elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "D");
  convertedElem =  convertedElem->FirstChildElement("elemC");
  EXPECT_TRUE(convertedElem);
  EXPECT_FALSE(convertedElem->FirstChildElement("elemD"));

  // Test moving from elem to attr
  TiXmlDocument xmlDoc2;
  xmlDoc2.Parse(stream.str().c_str());
  convertStream.str("");
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "     <from element='elemC::elemD'/>"
                << "     <to attribute='attrE'/>"
                << "   </move>"
                << " </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc2;
  convertXmlDoc2.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc2, &convertXmlDoc2);

  convertedElem =  xmlDoc2.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->Attribute("attrE"));
  std::string attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "D");
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->FirstChildElement("elemD"));

  // Test moving from attr to attr
  TiXmlDocument xmlDoc3;
  xmlDoc3.Parse(stream.str().c_str());
  convertStream.str("");
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "     <from attribute='elemC::attrC'/>"
                << "     <to attribute='attrE'/>"
                << "   </move>"
                << " </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc3;
  convertXmlDoc3.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc3, &convertXmlDoc3);

  convertedElem =  xmlDoc3.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->Attribute("attrE"));
  attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "C");
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");

  // Test moving from attr to elem
  TiXmlDocument xmlDoc4;
  xmlDoc4.Parse(stream.str().c_str());
  convertStream.str("");
  convertStream << "<convert name='elemA'>"
                << "  <convert name='elemB'>"
                << "    <move>"
                << "     <from attribute='elemC::attrC'/>"
                << "     <to element='elemE'/>"
                << "   </move>"
                << " </convert>"
                << "</convert>";
  TiXmlDocument convertXmlDoc4;
  convertXmlDoc4.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc4, &convertXmlDoc4);

  convertedElem =  xmlDoc4.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemB");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemE"));
  elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "C");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemC"));
  convertedElem =  convertedElem->FirstChildElement("elemC");
  EXPECT_TRUE(convertedElem);
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");

  // Test moving from elem to elem across multiple levels
  TiXmlDocument xmlDoc5;
  xmlDoc5.Parse(stream.str().c_str());
  convertStream.str("");
  convertStream << "<convert name='elemA'>"
                << "  <move>"
                << "   <from element='elemB::elemC::elemD'/>"
                << "   <to element='elemE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc5;
  convertXmlDoc5.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc5, &convertXmlDoc5);

  convertedElem =  xmlDoc5.FirstChildElement();
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  EXPECT_TRUE(convertedElem->FirstChildElement("elemB"));
  EXPECT_TRUE(convertedElem->FirstChildElement("elemE"));
  elemValue = convertedElem->FirstChildElement("elemE")->GetText();
  EXPECT_EQ(elemValue, "D");
  convertedElem =  convertedElem->FirstChildElement("elemB");
  EXPECT_TRUE(convertedElem);
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->FirstChildElement("elemD"));

    // Test moving from attr to attr across multiple levels
  TiXmlDocument xmlDoc6;
  xmlDoc6.Parse(stream.str().c_str());
  convertStream.str("");
  convertStream << "<convert name='elemA'>"
                << "  <move>"
                << "   <from attribute='elemB::elemC::attrC'/>"
                << "   <to attribute='attrE'/>"
                << "  </move>"
                << "</convert>";
  TiXmlDocument convertXmlDoc6;
  convertXmlDoc6.Parse(convertStream.str().c_str());
  sdf::Converter::Convert(&xmlDoc6, &convertXmlDoc6);

  convertedElem =  xmlDoc6.FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemA");
  attrValue = convertedElem->Attribute("attrE");
  EXPECT_EQ(attrValue, "C");
  convertedElem = convertedElem->FirstChildElement("elemB");
  EXPECT_TRUE(convertedElem);
  convertedElem =  convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemC");
  EXPECT_FALSE(convertedElem->Attribute("attrC"));
  convertedElem = convertedElem->FirstChildElement();
  EXPECT_TRUE(convertedElem);
  EXPECT_EQ(convertedElem->ValueStr(), "elemD");
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
