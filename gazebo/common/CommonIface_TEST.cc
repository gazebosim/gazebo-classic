/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <stdlib.h>
#include <gtest/gtest.h>
#include <string>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>

#include <ignition/common/Filesystem.hh>
#include <sdf/parser.hh>

#include "test_config.h"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/SystemPaths.hh"
#include "test/util.hh"

using namespace gazebo;

class CommonIface_TEST : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
/// \brief Test CommonIface::GetSHA1
TEST_F(CommonIface_TEST, GetSHA1)
{
  // Do not forget to update 'precomputedSHA1' if you modify the SHA1 input.
  std::string precomputedSHA1;
  std::string computedSHA1;
  std::string s;

  // Compute the SHA1 of the vector
  std::vector<float> v;
  for (int i = 0; i < 100; ++i)
    v.push_back(i);

  computedSHA1 = common::get_sha1<std::vector<float> >(v);
  precomputedSHA1 = "913283ec8502ba1423d38a7ea62cb8e492e87b23";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute the SHA1 of a string
  s = "Marty McFly: Wait a minute, Doc. Ah... Are you telling me that you"
      " built a time machine... out of a DeLorean?\n"
      "Dr. Emmett Brown: The way I see it, if you're gonna build a time"
      " machine into a car, why not do it with some style?";
  computedSHA1 = common::get_sha1<std::string>(s);
  precomputedSHA1 = "a370ddc4d61d936b2bb40f98bae061dc15fd8923";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute the SHA1 of an empty string
  s = "";
  computedSHA1 = common::get_sha1<std::string>(s);
  precomputedSHA1 = "da39a3ee5e6b4b0d3255bfef95601890afd80709";
  EXPECT_EQ(precomputedSHA1, computedSHA1);

  // Compute a bunch of SHA1's to verify consistent length
  for (unsigned i = 0; i < 100; ++i)
  {
    std::stringstream stream;
    stream << i << '\n';
    std::string sha = common::get_sha1<std::string>(stream.str());
    EXPECT_EQ(sha.length(), 40u);
  }
}

/////////////////////////////////////////////////
/// \brief Test the string tokenizer split() function.
TEST_F(CommonIface_TEST, split)
{
  auto tokens = common::split("abc/def", "/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  tokens = common::split("abc/def/", "/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  tokens = common::split("//abc/def///", "/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def");

  tokens = common::split("abc", "/");
  ASSERT_EQ(tokens.size(), 1u);
  EXPECT_EQ(tokens.at(0), "abc");

  tokens = common::split("//abc/def::123::567///", "/");
  ASSERT_EQ(tokens.size(), 2u);
  EXPECT_EQ(tokens.at(0), "abc");
  EXPECT_EQ(tokens.at(1), "def::123::567");
  tokens = common::split("//abc/def::123::567///", "::");
  ASSERT_EQ(tokens.size(), 3u);
  EXPECT_EQ(tokens.at(0), "//abc/def");
  EXPECT_EQ(tokens.at(1), "123");
  EXPECT_EQ(tokens.at(2), "567///");
}

/////////////////////////////////////////////////
/// \brief Test file operations
TEST_F(CommonIface_TEST, fileOps)
{
  EXPECT_FALSE(common::cwd().empty());
  EXPECT_TRUE(common::exists(common::cwd()));
  EXPECT_TRUE(common::isDirectory(common::cwd()));

  EXPECT_TRUE(common::isFile(__FILE__));
  EXPECT_FALSE(common::isDirectory(__FILE__));

  std::ofstream tmpOut("test.tmp");
  tmpOut << "Output" << std::endl;
  tmpOut.close();

  EXPECT_TRUE(common::copyFile("test.tmp", "test2.tmp"));
  EXPECT_TRUE(common::exists("test.tmp"));
  EXPECT_TRUE(common::exists("test2.tmp"));

  std::ifstream testIn("test.tmp");
  std::string testInContent((std::istreambuf_iterator<char>(testIn)),
                            (std::istreambuf_iterator<char>()));

  std::ifstream test2In("test2.tmp");
  std::string test2InContent((std::istreambuf_iterator<char>(test2In)),
                             (std::istreambuf_iterator<char>()));


  EXPECT_EQ(testInContent, test2InContent);

  EXPECT_TRUE(common::moveFile("test2.tmp", "test3.tmp"));
  EXPECT_FALSE(common::exists("test2.tmp"));
  EXPECT_TRUE(common::exists("test3.tmp"));

  std::ifstream test3In("test3.tmp");
  std::string test3InContent((std::istreambuf_iterator<char>(test3In)),
                             (std::istreambuf_iterator<char>()));

  EXPECT_EQ(testInContent, test3InContent);

  EXPECT_FALSE(common::copyFile("test3.tmp", "test3.tmp"));
  EXPECT_FALSE(common::copyFile("test3.tmp", "./test3.tmp"));

  std::remove("test.tmp");

  // This file shouldn't exist, but we'll try to remove just in case the
  // test failed.
  std::remove("test2.tmp");

  std::remove("test3.tmp");
}

/////////////////////////////////////////////////
/// \brief Test file operations
TEST_F(CommonIface_TEST, moreFileOps)
{
  EXPECT_FALSE(common::copyFile("__wrong__.tmp", "test2.tmp"));
  EXPECT_TRUE(!common::exists("test2.tmp"));
  EXPECT_FALSE(common::copyFile("test.tmp", "__wrong_dir__/__wrong__.tmp"));
  EXPECT_TRUE(!common::exists("__wrong_dir__"));

  EXPECT_FALSE(common::moveFile("__wrong__.tmp", "test3.tmp"));
  EXPECT_TRUE(!common::exists("test3.tmp"));
  EXPECT_FALSE(common::moveFile("test2.tmp", "__wrong_dir__/__wrong__.tmp"));
  EXPECT_TRUE(!common::exists("__wrong_dir__"));
}

/////////////////////////////////////////////////
TEST_F(CommonIface_TEST, replaceAll)
{
  std::string orig = "//abcd/efg///ijk////lm/////////////nop//";

  // No change should occur
  std::string result = common::replaceAll(orig, "//", "//");
  EXPECT_EQ(result, orig);
  result = common::replaceAll(orig, "/", "/");
  EXPECT_EQ(result, orig);

  result = common::replaceAll(orig, "//", "::");
  EXPECT_EQ(result, "::abcd/efg::/ijk::::lm::::::::::::/nop::");

  result = common::replaceAll(result, "a", "aaaa");
  EXPECT_EQ(result, "::aaaabcd/efg::/ijk::::lm::::::::::::/nop::");

  result = common::replaceAll(result, "::aaaa", " ");
  EXPECT_EQ(result, " bcd/efg::/ijk::::lm::::::::::::/nop::");

  result = common::replaceAll(result, " ", "_");
  EXPECT_EQ(result, "_bcd/efg::/ijk::::lm::::::::::::/nop::");

  std::string spaces = " 1  2   3    4     5      6       7 ";
  result = common::replaceAll(spaces, " ", "_");
  EXPECT_EQ(result, "_1__2___3____4_____5______6_______7_");

  result = common::replaceAll(spaces, "  ", "_");
  EXPECT_EQ(result, " 1_2_ 3__4__ 5___6___ 7 ");

  std::string test = "12345555675";
  common::replaceAll(test, test, "5", "*");
  EXPECT_EQ(test, "1234****67*");
}

/////////////////////////////////////////////////
TEST_F(CommonIface_TEST, directoryOps)
{
  // Cleanup test directory.
  common::SystemPaths *paths = common::SystemPaths::Instance();
  boost::filesystem::path testPath(paths->DefaultTestPath());
  boost::filesystem::remove_all(testPath);
  boost::filesystem::create_directories(testPath);

  boost::filesystem::path src = testPath / "src";
  boost::filesystem::path dest = testPath / "dest";
  boost::filesystem::create_directories(src);
  boost::filesystem::path srcFilePath = src / "test.txt";
  std::ofstream srcFile(srcFilePath.string());
  srcFile << "This is a test file!" << std::endl;
  srcFile.close();
  EXPECT_FALSE(boost::filesystem::exists(dest));
  EXPECT_TRUE(boost::filesystem::exists(srcFilePath));

  // src exists, dest doesn't
  EXPECT_TRUE(common::copyDir(src, dest));
  EXPECT_TRUE(boost::filesystem::exists(dest));

  // src not exists
  boost::filesystem::remove_all(src);
  EXPECT_FALSE(common::copyDir(src, dest));

  // dest exists with nonempty contents
  boost::filesystem::create_directories(src);
  boost::filesystem::path srcFile2Path = src / "test2.txt";
  std::ofstream srcFile2(srcFile2Path.string());
  srcFile2 << "This is a 2nd test file!" << std::endl;
  srcFile2.close();
  EXPECT_FALSE(boost::filesystem::exists(srcFilePath));
  boost::filesystem::path destFilePath = dest / "test.txt";
  boost::filesystem::path destFile2Path = dest / "test2.txt";
  EXPECT_TRUE(boost::filesystem::exists(destFilePath));
  EXPECT_TRUE(boost::filesystem::exists(srcFile2Path));

  EXPECT_TRUE(common::copyDir(src, dest));
  EXPECT_TRUE(common::exists(destFile2Path.string()));
  EXPECT_FALSE(common::exists(destFilePath.string()));
}

/////////////////////////////////////////////////
TEST_F(CommonIface_TEST, asFullPath)
{
  auto src = std::string(PROJECT_SOURCE_PATH);

  const auto relativeUri = ignition::common::joinPaths("data", "box.dae");
  const auto absoluteUri = ignition::common::joinPaths(src, "test", "data",
      "box.dae");
  const auto schemeUri{"https://website.com/collision.dae"};

  // Empty path
  {
    const std::string path{""};

    EXPECT_EQ(relativeUri, common::asFullPath(relativeUri, path));
    EXPECT_EQ(absoluteUri, common::asFullPath(absoluteUri, path));
    EXPECT_EQ(schemeUri, common::asFullPath(schemeUri, path));
  }

  // Data string
  {
    const std::string path{"data-string"};

    EXPECT_EQ(relativeUri, common::asFullPath(relativeUri, path));
    EXPECT_EQ(absoluteUri, common::asFullPath(absoluteUri, path));
    EXPECT_EQ(schemeUri, common::asFullPath(schemeUri, path));
  }

  {
    // Absolute path
    const auto path = ignition::common::joinPaths(src, "test", "path");

    // Directory
    EXPECT_EQ(ignition::common::joinPaths(src, "test", "data", "box.dae"),
        common::asFullPath(relativeUri, path));
    EXPECT_EQ(absoluteUri, common::asFullPath(absoluteUri, path));
    EXPECT_EQ(schemeUri, common::asFullPath(schemeUri, path));

    // File
    const auto filePath = ignition::common::joinPaths(src, "test", "file.sdf");
    EXPECT_EQ(ignition::common::joinPaths(src, "test", "data", "box.dae"),
        common::asFullPath(relativeUri, filePath));
    EXPECT_EQ(absoluteUri, common::asFullPath(absoluteUri, filePath));
    EXPECT_EQ(schemeUri, common::asFullPath(schemeUri, filePath));
  }
}

/////////////////////////////////////////////////
TEST_F(CommonIface_TEST, fullPathsMesh)
{
  auto path = ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test");

  std::string filePath = ignition::common::joinPaths(path, "model.sdf");
  std::string relativePath = ignition::common::joinPaths("data", "box.dae");

  std::stringstream ss;
  ss << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='mesh_model'>"
    << "<link name ='link'>"
    << "  <collision name ='collision'>"
    << "    <geometry>"
    << "      <mesh>"
    << "        <uri>" << relativePath << "</uri>"
    << "      </mesh>"
    << "    </geometry>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << "      <mesh><uri>" << relativePath << "</uri></mesh>"
    << "    </geometry>"
    << "  </visual>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  auto modelElem = std::make_shared<sdf::Element>();
  sdf::initFile("model.sdf", modelElem);
  sdf::readString(ss.str(), modelElem);

  // Before conversion
  ASSERT_NE(nullptr, modelElem);

  auto linkElem = modelElem->GetElement("link");
  ASSERT_NE(nullptr, linkElem);

  for (auto elemStr : {"collision", "visual"})
  {
    auto elem = linkElem->GetElement(elemStr);
    ASSERT_NE(nullptr, elem);

    auto geometryElem = elem->GetElement("geometry");
    ASSERT_NE(nullptr, geometryElem);

    auto meshElem = geometryElem->GetElement("mesh");
    ASSERT_NE(nullptr, meshElem);

    auto uriElem = meshElem->GetElement("uri");
    ASSERT_NE(nullptr, uriElem);

    EXPECT_EQ(relativePath, uriElem->Get<std::string>());
    uriElem->SetFilePath(filePath);
  }

  auto sdfString = modelElem->ToString("");

  // SDF element conversion
  common::convertToFullPaths(modelElem);

  ASSERT_NE(nullptr, modelElem);

  linkElem = modelElem->GetElement("link");
  ASSERT_NE(nullptr, linkElem);

  for (auto elemStr : {"collision", "visual"})
  {
    auto elem = linkElem->GetElement(elemStr);
    ASSERT_NE(nullptr, elem);

    auto geometryElem = elem->GetElement("geometry");
    ASSERT_NE(nullptr, geometryElem);

    auto meshElem = geometryElem->GetElement("mesh");
    ASSERT_NE(nullptr, meshElem);

    auto uriElem = meshElem->GetElement("uri");
    ASSERT_NE(nullptr, uriElem);

    EXPECT_EQ(ignition::common::joinPaths(path, relativePath),
        uriElem->Get<std::string>());
  }


  // SDF string conversion
  sdfString = "<sdf version='" + std::string(SDF_VERSION) + "'>\n" + sdfString +
      "\n</sdf>";
  common::convertToFullPaths(sdfString, modelElem->FilePath());
  sdf::readString(sdfString, modelElem);

  linkElem = modelElem->GetElement("link");
  ASSERT_NE(nullptr, linkElem);

  for (auto elemStr : {"collision", "visual"})
  {
    auto elem = linkElem->GetElement(elemStr);
    ASSERT_NE(nullptr, elem);

    auto geometryElem = elem->GetElement("geometry");
    ASSERT_NE(nullptr, geometryElem);

    auto meshElem = geometryElem->GetElement("mesh");
    ASSERT_NE(nullptr, meshElem);

    auto uriElem = meshElem->GetElement("uri");
    ASSERT_NE(nullptr, uriElem);

    EXPECT_EQ(ignition::common::joinPaths(path, relativePath),
        uriElem->Get<std::string>());
  }
}

/////////////////////////////////////////////////
TEST_F(CommonIface_TEST, fullPathsActor)
{
  auto path = ignition::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test");

  std::string filePath = ignition::common::joinPaths(path, "model.sdf");
  std::string relativePath = ignition::common::joinPaths("data", "box.dae");

  std::stringstream ss;
  ss << "<sdf version='" << SDF_VERSION << "'>"
    << "<actor name='actor_test'>"
    << "  <skin>"
    << "    <filename>" << relativePath << "</filename>"
    << "  </skin>"
    << "  <animation name='walk'>"
    << "    <filename>" << relativePath << "</filename>"
    << "  </animation>"
    << "</actor>"
    << "</sdf>";

  auto actorElem = std::make_shared<sdf::Element>();
  sdf::initFile("actor.sdf", actorElem);
  sdf::readString(ss.str(), actorElem);

  // Before conversion
  ASSERT_NE(nullptr, actorElem);

  auto skinElem = actorElem->GetElement("skin");
  ASSERT_NE(nullptr, skinElem);
  auto skinFilenameElem = skinElem->GetElement("filename");
  ASSERT_NE(nullptr, skinFilenameElem);
  EXPECT_EQ(relativePath, skinFilenameElem->Get<std::string>());
  skinFilenameElem->SetFilePath(filePath);

  auto animationElem = actorElem->GetElement("animation");
  ASSERT_NE(nullptr, animationElem);
  auto animationFilenameElem = animationElem->GetElement("filename");
  ASSERT_NE(nullptr, animationFilenameElem);
  EXPECT_EQ(relativePath, animationFilenameElem->Get<std::string>());
  animationFilenameElem->SetFilePath(filePath);

  // SDF element conversion
  common::convertToFullPaths(actorElem);
  ASSERT_NE(nullptr, actorElem);

  skinElem = actorElem->GetElement("skin");
  ASSERT_NE(nullptr, skinElem);
  skinFilenameElem = skinElem->GetElement("filename");
  ASSERT_NE(nullptr, skinFilenameElem);
  EXPECT_EQ(ignition::common::joinPaths(path, relativePath),
      skinFilenameElem->Get<std::string>());

  animationElem = actorElem->GetElement("animation");
  ASSERT_NE(nullptr, animationElem);
  animationFilenameElem = animationElem->GetElement("filename");
  ASSERT_NE(nullptr, animationFilenameElem);
  EXPECT_EQ(ignition::common::joinPaths(path, relativePath),
      animationFilenameElem->Get<std::string>());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
