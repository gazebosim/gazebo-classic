/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"
#include "test/util.hh"

using namespace gazebo;

class Exception : public gazebo::testing::AutoLogFixture { };

TEST_F(Exception, Exception)
{
  try
  {
    gzthrow("test");
  }
  catch(common::Exception &_e)
  {
    std::cout << "Exception[" << _e.GetErrorFile() << "]\n";
    std::cout << "Exception[" << _e.GetErrorStr() << "]\n";
  }
}

TEST_F(Exception, InternalError_DefaultConstructor_Throw)
{
  ASSERT_THROW(
    throw common::InternalError(),
    common::InternalError);
}

TEST_F(Exception, InternalError_FileLineMsgConstructor_Throw)
{
  ASSERT_THROW(
      throw common::InternalError(__FILE__, __LINE__, "test"),
      common::InternalError);
}

TEST_F(Exception, InternalAssertionError_AssertionConstructor_Throw)
{
  ASSERT_THROW(
      throw common::AssertionInternalError(__FILE__, __LINE__, "expression",
                                          "function", "msg"),
      common::AssertionInternalError);
}

#if defined(BOOST_DISABLE_ASSERTS) || defined(NDEBUG)
TEST_F(Exception, GZASSERT_Disabled_NoThrow)
{
  ASSERT_NO_THROW(
     GZ_ASSERT(true == false, "Assert thrown"));
}
#elif defined(BOOST_ENABLE_ASSERT_HANDLER)
TEST_F(Exception, GZASSERT_WithHandler_ThrowException)
{
  ASSERT_THROW(
    GZ_ASSERT(true == false, "Assert thrown"),
               common::AssertionInternalError);
}
#else
TEST_F(Exception, GZASSERT_Enabled_ThrowAssertion)
{
    ASSERT_DEATH(
      GZ_ASSERT(true == false, "Assert thrown"),
      ".*Assertion.*failed.*|.*Internal Program Error - assertion.*");
}
#endif


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
