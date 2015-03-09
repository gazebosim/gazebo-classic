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

#include "gazebo/gui/model/ExtrudeDialog.hh"
#include "gazebo/gui/model/ExtrudeDialog_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ExtrudeDialog_TEST::BadFilename()
{
  // Check that bad filenames don't break anything
  std::string bad("/not/a/file.svg");

  gazebo::gui::ExtrudeDialog *extrudeDialog =
      new gazebo::gui::ExtrudeDialog(bad);
  QVERIFY(extrudeDialog != NULL);

  delete extrudeDialog;
}

/////////////////////////////////////////////////
void ExtrudeDialog_TEST::GetSpinValues()
{
  // Check a good file
  std::string filePath = std::string(PROJECT_SOURCE_PATH);
  filePath += "/test/data/loader.svg";
  gazebo::gui::ExtrudeDialog *extrudeDialog =
      new gazebo::gui::ExtrudeDialog(filePath);

  // Get default thickness, change value and check new value
  double thickness = extrudeDialog->GetThickness();

  QCOMPARE(thickness, 0.1);
  QDoubleSpinBox *thicknessSpin =
      extrudeDialog->findChild<QDoubleSpinBox *>("thicknessSpin");
  QVERIFY(thicknessSpin != NULL);
  thicknessSpin->setValue(1.0);
  thickness = extrudeDialog->GetThickness();
  QCOMPARE(thickness, 1.0);

  // Get default resolution, change value and check new value
  double resolution = extrudeDialog->GetResolution();
  QCOMPARE(resolution, 3543.3);
  QDoubleSpinBox *resolutionSpin =
      extrudeDialog->findChild<QDoubleSpinBox *>("resolutionSpin");
  QVERIFY(resolutionSpin != NULL);
  resolutionSpin->setValue(1010.1);
  resolution = extrudeDialog->GetResolution();
  QCOMPARE(resolution, 1010.1);

  // Get default number of samples, change value and check new value
  unsigned int samples = extrudeDialog->GetSamples();
  QCOMPARE(samples, (unsigned int)5);
  QSpinBox *samplesSpin =
      extrudeDialog->findChild<QSpinBox *>("samplesSpin");
  QVERIFY(samplesSpin != NULL);
  samplesSpin->setValue(20);
  samples = extrudeDialog->GetSamples();
  QCOMPARE(samples, (unsigned int)20);
}

// Generate a main function for the test
QTEST_MAIN(ExtrudeDialog_TEST)
