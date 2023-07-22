/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "gazebo/common/CommonIface.hh"
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

/////////////////////////////////////////////////
#ifdef _WIN32
static int setenv(const char *envname, const char *envval, int overwrite)
{
  char *original = getenv(envname);
  if (!original || !!overwrite)
  {
    std::string envstring = std::string(envname) + "=" + envval;
    return _putenv(envstring.c_str());
  }
  return 0;
}
#endif

class SDFLogsTest : public ServerFixture
{
  public: void SetUp()
  {
    const boost::filesystem::path home = common::getEnv(HOMEDIR);
    boost::filesystem::path log_path("/.gazebo/server-11345/default.log");
    path = home / log_path;
  }

  public: void EXPECT_LOG_STRING(const std::string expected_text)
  {
    EXPECT_TRUE(log_string_search(expected_text)) <<
      "The text '" + expected_text + "'" +
      " was not found in the log. The test expects it to be there";
  }

  public: void EXPECT_NO_ERR_IN_LOG()
  {
    EXPECT_NO_LOG_STRING("[Err] ");
  }

  public: void EXPECT_NO_LOG_STRING(const std::string no_expected_text)
  {
    EXPECT_FALSE(log_string_search(no_expected_text)) <<
      "The text '" + no_expected_text + "'" +
      " was found in the log. The test does not expect it be there";
  }

  public: void EXPECT_SDF_ERR_IN_LOG()
  {
    EXPECT_LOG_STRING("[Err] ");
    std::string sdfErrorString = "SDF is not valid";
    EXPECT_LOG_STRING(sdfErrorString);
  }

  private: bool log_string_search(const std::string expected_text)
  {
    // Open the log file, and read back the string
    std::ifstream ifs(path.string().c_str(), std::ios::in);
    std::string loggedString;

    while (!ifs.eof())
    {
      std::string line;
      std::getline(ifs, line);
      loggedString += line;
    }

    return loggedString.find(expected_text) != std::string::npos;
  }

  private: boost::filesystem::path path;
};

/////////////////////////////////////////////////
TEST_F(SDFLogsTest, EmptyWorldNoErrors)
{
  Load("worlds/empty.world");
  EXPECT_NO_ERR_IN_LOG();
}

/////////////////////////////////////////////////
TEST_F(SDFLogsTest, DuplicateSiblingSameType16)
{
  Load("worlds/test_sdf16_err_sibling_same_type.world");
  EXPECT_SDF_ERR_IN_LOG();
}

/////////////////////////////////////////////////
TEST_F(SDFLogsTest, DuplicateSiblingSameTypeDisabled16)
{
  setenv("GAZEBO11_BACKWARDS_COMPAT_WARNINGS_ERRORS", "", 1);
  Load("worlds/test_sdf16_err_sibling_same_type.world");
  EXPECT_NO_ERR_IN_LOG();
#ifdef _WIN32
 _putenv("GAZEBO11_BACKWARDS_COMPAT_WARNINGS_ERRORS");
#else
  unsetenv("GAZEBO11_BACKWARDS_COMPAT_WARNINGS_ERRORS");
#endif
}

/////////////////////////////////////////////////
TEST_F(SDFLogsTest, DuplicateSiblingDifferentType16)
{
  Load("worlds/test_sdf16_err_sibling_different_type.world");
  // 1.6 SDF does NOT enforce different names for different types
  EXPECT_NO_ERR_IN_LOG();
}

TEST_F(SDFLogsTest, DuplicateSiblingDifferentType17)
{
  // 1.7+ SDF does enforce different names for different types
  Load("worlds/test_sdf17_err_sibling_different_type.world");
  EXPECT_SDF_ERR_IN_LOG();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
