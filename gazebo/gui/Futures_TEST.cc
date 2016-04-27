/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/gui/Futures.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/Futures_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void Futures_TEST::Verify()
{
  std::cerr << "Loading shapes world\n";
  this->Load("worlds/shapes.world", false, false, false);

  std::cerr << "Getting result\n";
  bool result = gazebo::gui::Futures::introspectionClientFuture.valid();
  std::cerr << "Result is[" << result << "]\n";
  QVERIFY(result);
  gazebo::gui::Futures::introspectionClientFuture.wait();
  std::cerr << "After introspection client wait\n";
}

// Generate a main function for the test
QTEST_MAIN(Futures_TEST)
