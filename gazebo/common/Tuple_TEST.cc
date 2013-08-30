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

typedef std::tr1::tuple<std::string, std::string> const_char2;
class TupleTest : public ::testing::TestWithParam<const_char2>
{
  public: virtual void SetUp()
          {
            std::tr1::tie(get0, get1) = GetParam();
          }
  public: std::string get0;
  public: std::string get1;
};

TEST_P(TupleTest, Hello)
{
  std::cout << "get<0> " << std::tr1::get<0>(GetParam()) << std::endl;
  std::cout << "get<1> " << std::tr1::get<1>(GetParam()) << std::endl;

  std::cout << "this->get0 " << this->get0 << std::endl;
  std::cout << "this->get1 " << this->get1 << std::endl;
}

INSTANTIATE_TEST_CASE_P(TestRuns, TupleTest, ::testing::Combine(
  ::testing::Values("AAA", "BBB", "CCC", "DDD"),
  ::testing::Values("111", "222", "333", "444")
));

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
