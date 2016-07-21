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
#include <sys/types.h>
#include <sys/stat.h>
#include "gazebo/gui/plot/PlotCanvas.hh"
#include "gazebo/gui/plot/ExportDialog.hh"
#include "gazebo/gui/plot/ExportDialog_TEST.hh"

/////////////////////////////////////////////////
void ExportDialog_TEST::VerifyButtons(
    gazebo::gui::ExportDialog *_exportDialog, const bool _enabled)
{
  QList<QPushButton *> buttons = _exportDialog->findChildren<QPushButton *>(
      "materialFlat");

  QVERIFY(buttons.count() == 2);
  if (buttons.at(0)->text() == QString("&Export to"))
  {
    QVERIFY(buttons.at(0)->isEnabled() == _enabled);
    QVERIFY(buttons.at(1)->isEnabled());
  }
  else if (buttons.at(1)->text() == QString("&Export to"))
  {
    QVERIFY(buttons.at(1)->isEnabled() == _enabled);
    QVERIFY(buttons.at(0)->isEnabled());
  }
  else
  {
    QFAIL("Invalid buttons on export dialog");
  }
}

/////////////////////////////////////////////////
void ExportDialog_TEST::Select(gazebo::gui::ExportDialog *_exportDialog,
    const bool _all)
{
  QList<QAction *> actions = _exportDialog->findChildren<QAction *>();

  QAction *selectAll = nullptr;
  QAction *selectNone = nullptr;
  for (int i = 0; i < actions.count(); ++i)
  {
    std::string txt = actions.at(i)->text().toStdString();
    if (txt == "Select all")
      selectAll = actions.at(i);
    if (txt == "Clear selection")
      selectNone = actions.at(i);
  }
  QVERIFY(selectAll != nullptr);
  QVERIFY(selectNone != nullptr);

  if (_all)
    selectAll->trigger();
  else
    selectNone->trigger();

  this->ProcessEventsAndDraw();
}

/////////////////////////////////////////////////
void ExportDialog_TEST::Empty()
{
  this->Load("worlds/empty.world");

  std::list<gazebo::gui::PlotCanvas *> plots;

  // Create a new plot canvas widget
  gazebo::gui::ExportDialog *exportDialog =
    new gazebo::gui::ExportDialog(nullptr, plots);

  QVERIFY(exportDialog != nullptr);

  exportDialog->show();

  auto listView = exportDialog->findChild<QListView *>("plotExportListView");
  QVERIFY(listView != nullptr);
  QVERIFY(listView->model()->columnCount() == 0);
  QVERIFY(listView->model()->rowCount() == 0);

  this->VerifyButtons(exportDialog, false);

  exportDialog->hide();
  delete exportDialog;
}

/////////////////////////////////////////////////
void ExportDialog_TEST::OnePlot()
{
  this->Load("worlds/empty.world");

  std::list<gazebo::gui::PlotCanvas *> plots;

  // Create a new plot canvas widget
  gazebo::gui::PlotCanvas *plotCanvas = new gazebo::gui::PlotCanvas(nullptr);
  QVERIFY(plotCanvas != nullptr);
  plotCanvas->show();

  // there should be an empty plot
  QCOMPARE(plotCanvas->PlotCount(), 1u);

  plots.push_back(plotCanvas);

  // Create a new plot canvas widget
  gazebo::gui::ExportDialog *exportDialog =
    new gazebo::gui::ExportDialog(nullptr, plots);

  QVERIFY(exportDialog != nullptr);

  exportDialog->show();
  this->ProcessEventsAndDraw();

  // Get the list view, which holds the plotcanvas items.
  auto listView = exportDialog->findChild<QListView *>("plotExportListView");
  QVERIFY(listView != nullptr);
  QCOMPARE(listView->model()->columnCount(), 1);
  QCOMPARE(listView->model()->rowCount(), 1);

  // Verify that the export button is disabled.
  this->VerifyButtons(exportDialog, false);

  // Get the rectangle for the plot item in the list view
  QRect rect = listView->visualRect(listView->model()->index(0, 0));

  // Click the list item
  QTest::mouseClick(listView->viewport(), Qt::LeftButton,
      Qt::NoModifier, rect.center());
  this->ProcessEventsAndDraw();

  // The export button should now be enabled.
  this->VerifyButtons(exportDialog, true);

  // Click the list item again
  QTest::mouseClick(listView->viewport(), Qt::LeftButton,
      Qt::NoModifier, rect.center());
  this->ProcessEventsAndDraw();

  // The export button should now be disabled.
  this->VerifyButtons(exportDialog, false);

  // Select all
  this->Select(exportDialog, true);
  this->VerifyButtons(exportDialog, true);

  // Select none
  this->Select(exportDialog, false);
  this->VerifyButtons(exportDialog, false);

  exportDialog->hide();
  delete exportDialog;

  plotCanvas->hide();
  delete plotCanvas;
}

/////////////////////////////////////////////////
void ExportDialog_TEST::ExportPDF()
{
  // Create a new plot canvas widget
  gazebo::gui::PlotCanvas *plotCanvas = new gazebo::gui::PlotCanvas(nullptr);
  QVERIFY(plotCanvas != nullptr);
  plotCanvas->show();

  // Add a plot to the canvas
  int index = plotCanvas->AddPlot();
  QCOMPARE(index, 0);

  // Using linux filesystem commands for this test.
  // \todo: Test on mac.
#ifdef __linux__
  std::string outputDir = "/tmp/_gz_test_plot_pdf_export_";

  // Create a directory to store the PDF file
  mkdir(outputDir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);

  // Export the plot to pdf
  plotCanvas->Export(outputDir.c_str(), gazebo::gui::FileType::PDFFile);

  DIR *dir = nullptr;
  struct dirent entry;
  struct dirent *result = nullptr;
  bool foundFile = false;
  if ((dir = opendir(outputDir.c_str())) != nullptr)
  {
    // Check the directory contents for the correct pdf file.
    while (readdir_r(dir, &entry, &result) == 0 && result != nullptr)
    {
      std::string filename = result->d_name;
      std::cout << "Filename[" << filename << "]\n";
      if (filename == plotCanvas->Title() + ".pdf")
      {
        foundFile = true;
        std::string fullPath = outputDir + "/" + filename;
        // Remove the file
        remove(fullPath.c_str());
      }
    }
    closedir(dir);
  }

  // Remove the directory
  rmdir(outputDir.c_str());
  QVERIFY(foundFile);
#endif
}

/////////////////////////////////////////////////
void ExportDialog_TEST::ExportCSV()
{
  // Load a world with the box model
  this->Load("worlds/shapes.world");

  // Create a new plot canvas widget
  gazebo::gui::PlotCanvas *plotCanvas = new gazebo::gui::PlotCanvas(nullptr);
  QVERIFY(plotCanvas != nullptr);
  plotCanvas->show();

  // Add a plot to the canvas
  int index = plotCanvas->AddPlot();
  QCOMPARE(index, 1);

  // Add a variable to the plot
  std::string var = std::string("data://world/default/model/box") +
    "?p=pose3d/world_pose/vector3d/position/double/x";
  plotCanvas->AddVariable(var, index);
  plotCanvas->SetVariableLabel(index, "test");

  // Collect some data
  gazebo::common::Time::MSleep(500);

  // Using linux filesystem commands for this test.
  // \todo: Test on mac.
#ifdef __linux__
  std::string outputDir = "/tmp/_gz_test_plot_csv_export_";

  // Create a directory to store the PDF file
  mkdir(outputDir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);

  // Export the plot to csv
  plotCanvas->Export(outputDir.c_str(), gazebo::gui::FileType::CSVFile);

  DIR *dir = nullptr;
  struct dirent entry;
  struct dirent *result = nullptr;
  bool foundFile = false;
  if ((dir = opendir(outputDir.c_str())) != nullptr)
  {
    // Check the directory contents for the correct csv file.
    while (readdir_r(dir, &entry, &result) == 0 && result != nullptr)
    {
      std::string filename = result->d_name;
      if (filename == plotCanvas->Title() + "-box:world_pose_position_x.csv")
      {
        foundFile = true;
        std::string fullPath = outputDir + "/" + filename;
        // Remove the file
        remove(fullPath.c_str());
      }
    }
    closedir(dir);
  }

  // Remove the directory
  rmdir(outputDir.c_str());
  QVERIFY(foundFile);
#endif
}

// Generate a main function for the test
QTEST_MAIN(ExportDialog_TEST)
