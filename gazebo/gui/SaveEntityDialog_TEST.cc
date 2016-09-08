/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/SaveEntityDialog.hh"
#include "gazebo/gui/SaveEntityDialog_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void SaveEntityDialogTestHelper::CheckFileDialog()
{
  QVERIFY(this->dialog);

  QFileDialog *fileDialog = this->dialog->findChild<QFileDialog *>();
  QVERIFY(fileDialog);

  // set default path to home dir.
  fileDialog->setDirectory(QDir::homePath());

  // hit enter to close dialog
  QTest::keyClick(fileDialog, Qt::Key_Enter);

  QVERIFY(!fileDialog->isVisible());
}

/////////////////////////////////////////////////
void SaveEntityDialog_TEST::SaveLocation()
{
  gazebo::gui::SaveEntityDialog *saveDialog = new gazebo::gui::SaveEntityDialog;
  QCoreApplication::processEvents();

  // Set the model name
  std::string modelName = "model name";
  saveDialog->SetModelName(modelName);

  // Get folder name from model name
  std::string folderName = saveDialog->GetFolderNameFromModelName(modelName);
  const std::string expectedFolderName("model_name");
  std::cout << "folderName:         " << folderName << std::endl;
  std::cout << "expectedFolderName: " << expectedFolderName << std::endl;
  QVERIFY(folderName == expectedFolderName);

  // find the browse button
  QList<QPushButton *> pushButtons = saveDialog->findChildren<QPushButton *>();
  QVERIFY(!pushButtons.empty());
  QPushButton *browseButton = NULL;
  for (int i = 0; i < pushButtons.size(); ++i)
  {
    QPushButton *button = pushButtons[i];
    QVERIFY(button);
    if (button->text().toLower().toStdString() == "browse")
      browseButton = button;
  }
  QVERIFY(browseButton);

  // set a path in the browse file dialog and verify value
  SaveEntityDialogTestHelper helper;
  helper.dialog = saveDialog;
  QTimer::singleShot(0, &helper, SLOT(CheckFileDialog()));
  browseButton->click();
  const std::string actualSaveLocation(saveDialog->GetSaveLocation());
  const std::string expectedSaveLocation(
    QDir::homePath().toStdString() +  "/"  + folderName);
  std::cout << "actualSaveLocation:   " << actualSaveLocation << std::endl;
  std::cout << "expectedSaveLocation: " << expectedSaveLocation << std::endl;
  QVERIFY(actualSaveLocation == expectedSaveLocation);

  delete saveDialog;
}

// Generate a main function for the test
QTEST_MAIN(SaveEntityDialog_TEST)
