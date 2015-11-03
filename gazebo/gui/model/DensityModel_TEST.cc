/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/gui/model/DensityModel.hh"
#include "gazebo/gui/model/DensityModel_TEST.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
void DensityModel_TEST::Init()
{
  DensityModel *model = DensityModel::Instance();

  QVERIFY(model != NULL);
  QVERIFY(!model->Entries().empty());
}

/////////////////////////////////////////////////
void DensityModel_TEST::Accessors()
{
  DensityModel *model = DensityModel::Instance();
  {
    const DensityEntry *entry = model->EntryByDesc("Aluminum");
    QVERIFY(entry != NULL);
    QVERIFY(entry->desc == "Aluminum");
  }
  {
    const DensityEntry *entry = model->EntryByDesc("Notfoundium");
    QVERIFY(entry == NULL);
  }
  {
    const DensityEntry *entry = model->EntryByValue(19300.0);
    QVERIFY(entry != NULL);
    QVERIFY(entry->desc == "Tungsten");
  }
  {
    const DensityEntry *entry = model->EntryByValue(1001001.001);
    QVERIFY(entry == NULL);
  }
}

// Generate a main function for the test
QTEST_MAIN(DensityModel_TEST)
