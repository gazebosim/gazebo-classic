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
#include "gazebo/sdf/interface/parser_urdf.hh"

class URDFParser : public ::testing::Test 
{
  public:
      TiXmlDocument get_empty()
      {
          return TiXmlDocument(); 
      }

      std::string get_minimal_urdf_txt()
      {
          std::ostringstream stream;
          stream << "<robot name='test_robot'>"
                 << "<link name='link1' />"
                 << "</robot>";
          return stream.str();
      }

      std::string get_minimal_sdf_txt()
      {
          std::ostringstream stream;
          stream << "<sdf version='1.4'>"
                 << "<model name='test_robot'>"
                 << "<pose>0 0 0 0 -0 0</pose>"
                 << "</model>"
                 << "</sdf>";
          return stream.str();
      }

      std::string get_full_model_urdf_txt()
      {
          std::ostringstream stream;
          stream << "<robot name='test_robot'>"
                 << "<link name='link1' />"
                 << "  <link name='link2' />"
                 << "  <link name='link3' />"
                 << "  <link name='link4' />"
                 << "  <joint name='joint1' type='continuous'>"
                 << "    <parent link='link1'/>"
                 << "    <child link='link2'/>"
                 << "  </joint>"
                 << "  <joint name='joint2' type='continuous'>"
                 << "    <parent link='link1'/>"
                 << "    <child link='link3'/>"
                 << "  </joint>"
                 << "  <joint name='joint3' type='continuous'>"
                 << "    <parent link='link3'/>"
                 << "    <child link='link4'/>"
                 << "  </joint>"
                 << "</robot>";
          return stream.str();
      }

      TiXmlDocument convert_str_to_xml(const std::string urdf)
      {
          TiXmlDocument tmp;
          tmp.Parse(urdf.c_str());
          return tmp;
      }
       
    protected:
        urdf2gazebo::URDF2Gazebo parser_;
};

/* By design, errors are only reported in std output */
TEST_F(URDFParser, InitModelDoc_EmptyDoc_NoThrow)
{
   ASSERT_NO_THROW(
     TiXmlDocument doc = get_empty();
     TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
   );
}

TEST_F(URDFParser, InitModelDoc_BasicModel_NoThrow)
{
    ASSERT_NO_THROW(
      std::string urdf = get_minimal_urdf_txt();
      TiXmlDocument doc = convert_str_to_xml(urdf);
      TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
    );
}

TEST_F(URDFParser, ParseResults_BasicModel_ParseEqualToModel)
{
   // URDF -> SDF
   std::string urdf = get_minimal_urdf_txt();
   TiXmlDocument doc = convert_str_to_xml(urdf);
   TiXmlDocument sdf_result = parser_.InitModelDoc(&doc);
   std::string sdf_result_str;
   sdf_result_str << sdf_result;

   // SDF -> SDF
   std::string sdf = get_minimal_sdf_txt();
   TiXmlDocument sdf_doc = convert_str_to_xml(sdf);
   std::string sdf_same_result_str;
   sdf_same_result_str << sdf_doc;

   ASSERT_EQ(sdf_same_result_str, sdf_result_str);
}
