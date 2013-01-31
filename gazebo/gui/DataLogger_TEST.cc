/*
 * Copyright 2011 Nate Koenig
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

#include "gazebo/gui/DataLogger.hh"
#include "DataLogger_TEST.moc"

/////////////////////////////////////////////////
void DataLogger_TEST::RecordButton()
{
  // Create a new data logger widget
  gazebo::gui::DataLogger *dataLogger = new gazebo::gui::DataLogger;

  // Get the record button
  QToolButton *recordButton = dataLogger->findChild<QToolButton*>(
      "dataLoggerRecordButton");

  // Toggle the record button, which starts logging.
  recordButton->toggle();

  // Wait for a log status return message
  while (dataLogger->GetDestination().empty())
  {
    // The following line tell QT to process its events. This is vital for
    // all tests, but it must be run in the main thread.
    QCoreApplication::processEvents();
    gazebo::common::Time::MSleep(100);
  }

  // Make sure the destination log file is correct.
  QVERIFY(dataLogger->GetDestination().find("test/state.log") !=
          std::string::npos);
}

// Generate a main function.
QTEST_MAIN(DataLogger_TEST)
