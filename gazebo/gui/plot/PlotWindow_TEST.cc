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

#include "gazebo/gui/plot/PlotCanvas.hh"
#include "gazebo/gui/plot/PlotWindow.hh"
#include "gazebo/gui/plot/PlotWindow_TEST.hh"

/////////////////////////////////////////////////
void PlotWindow_TEST::AddRemoveCanvas()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create a new plot window widget
  gazebo::gui::PlotWindow *plotWindow = new gazebo::gui::PlotWindow(nullptr);

  QVERIFY(plotWindow != nullptr);

  plotWindow->show();

  // there should be an empty canvas
  QCOMPARE(plotWindow->CanvasCount(), 1u);

  // add canvas
  gazebo::gui::PlotCanvas *canvas01 = plotWindow->AddCanvas();
  QVERIFY(canvas01 != nullptr);
  QCOMPARE(plotWindow->CanvasCount(), 2u);

  gazebo::gui::PlotCanvas *canvas02 = plotWindow->AddCanvas();
  QVERIFY(canvas02 != nullptr);
  QCOMPARE(plotWindow->CanvasCount(), 3u);

  gazebo::gui::PlotCanvas *canvas03 = plotWindow->AddCanvas();
  QVERIFY(canvas03 != nullptr);
  QCOMPARE(plotWindow->CanvasCount(), 4u);

  // remove canvas
  plotWindow->RemoveCanvas(canvas01);
  QCOMPARE(plotWindow->CanvasCount(), 3u);

  plotWindow->RemoveCanvas(canvas02);
  QCOMPARE(plotWindow->CanvasCount(), 2u);

  // remove already removed canvas
  plotWindow->RemoveCanvas(canvas02);
  QCOMPARE(plotWindow->CanvasCount(), 2u);

  // remove last canvas
  plotWindow->RemoveCanvas(canvas03);
  QCOMPARE(plotWindow->CanvasCount(), 1u);

  // check we can add more canvases
  gazebo::gui::PlotCanvas *canvas04 = plotWindow->AddCanvas();
  QVERIFY(canvas04 != nullptr);
  QCOMPARE(plotWindow->CanvasCount(), 2u);

  // clear canvases
  plotWindow->Clear();
  QCOMPARE(plotWindow->CanvasCount(), 0u);

  plotWindow->hide();
  delete plotWindow;
}

// Generate a main function for the test
QTEST_MAIN(PlotWindow_TEST)
