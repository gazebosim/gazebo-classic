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

#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/PlotCanvas.hh"
#include "gazebo/gui/plot/PlotCanvas_TEST.hh"

/////////////////////////////////////////////////
void PlotCanvas_TEST::AddRemovePlot()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create a new plot canvas widget
  gazebo::gui::PlotCanvas *plotCanvas = new gazebo::gui::PlotCanvas(nullptr);

  QVERIFY(plotCanvas != nullptr);

  plotCanvas->show();

  // there should be an empty plot
  QCOMPARE(plotCanvas->PlotCount(), 1u);

  // add plot
  unsigned int plot01 = plotCanvas->AddPlot();
  QCOMPARE(plotCanvas->PlotCount(), 2u);

  unsigned int plot02 = plotCanvas->AddPlot();
  QCOMPARE(plotCanvas->PlotCount(), 3u);

  unsigned int plot03 = plotCanvas->AddPlot();
  QCOMPARE(plotCanvas->PlotCount(), 4u);

  // remove plot
  plotCanvas->RemovePlot(plot01);
  QCOMPARE(plotCanvas->PlotCount(), 3u);

  plotCanvas->RemovePlot(plot02);
  QCOMPARE(plotCanvas->PlotCount(), 2u);

  // remove already removed plot
  plotCanvas->RemovePlot(plot02);
  QCOMPARE(plotCanvas->PlotCount(), 2u);

  // remove last plot
  plotCanvas->RemovePlot(plot03);
  QCOMPARE(plotCanvas->PlotCount(), 1u);

  // check we can add more plots
  plotCanvas->AddPlot();
  QCOMPARE(plotCanvas->PlotCount(), 2u);

  plotCanvas->hide();
  delete plotCanvas;
}

/////////////////////////////////////////////////
void PlotCanvas_TEST::AddRemoveVariable()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create a new plot canvas widget
  gazebo::gui::PlotCanvas *plotCanvas = new gazebo::gui::PlotCanvas(nullptr);
  QVERIFY(plotCanvas != nullptr);

  plotCanvas->show();

  // there should be an empty plot
  QCOMPARE(plotCanvas->PlotCount(), 1u);

  // add variable to first plot
  unsigned int var01 = plotCanvas->AddVariable("var01");
  QCOMPARE(plotCanvas->PlotCount(), 1u);
  QVERIFY(plotCanvas->PlotByVariable(var01) !=
      gazebo::gui::PlotCanvas::EmptyPlot);

  // add another variable - this creates a new plot
  unsigned int var02 = plotCanvas->AddVariable("var02");
  QCOMPARE(plotCanvas->PlotCount(), 2u);
  QVERIFY(plotCanvas->PlotByVariable(var02) !=
      gazebo::gui::PlotCanvas::EmptyPlot);

  // add one more variable
  unsigned int var03 = plotCanvas->AddVariable("var03");
  QCOMPARE(plotCanvas->PlotCount(), 3u);
  QVERIFY(plotCanvas->PlotByVariable(var03) !=
      gazebo::gui::PlotCanvas::EmptyPlot);

  // remove variable
  plotCanvas->RemoveVariable(var01);
  QCOMPARE(plotCanvas->PlotCount(), 2u);
  QVERIFY(plotCanvas->PlotByVariable(var01) ==
      gazebo::gui::PlotCanvas::EmptyPlot);

  plotCanvas->RemoveVariable(var02);
  QCOMPARE(plotCanvas->PlotCount(), 1u);
  QVERIFY(plotCanvas->PlotByVariable(var02) ==
      gazebo::gui::PlotCanvas::EmptyPlot);

  // remove already removed plot
  plotCanvas->RemoveVariable(var02);
  QCOMPARE(plotCanvas->PlotCount(), 1u);
  QVERIFY(plotCanvas->PlotByVariable(var02) ==
      gazebo::gui::PlotCanvas::EmptyPlot);

  // remove last variable - this should leave an empty plot in the canvas
  plotCanvas->RemoveVariable(var03);
  QCOMPARE(plotCanvas->PlotCount(), 1u);
  QVERIFY(plotCanvas->PlotByVariable(var03) ==
      gazebo::gui::PlotCanvas::EmptyPlot);

  // check we can add more variables - should now have one plot with variable
  unsigned int var04 = plotCanvas->AddVariable("var04");
  QCOMPARE(plotCanvas->PlotCount(), 1u);
  QVERIFY(plotCanvas->PlotByVariable(var04) !=
      gazebo::gui::PlotCanvas::EmptyPlot);

  plotCanvas->hide();
  delete plotCanvas;
}

/////////////////////////////////////////////////
void PlotCanvas_TEST::VariableLabel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create a new plot canvas widget
  gazebo::gui::PlotCanvas *plotCanvas = new gazebo::gui::PlotCanvas(nullptr);
  QVERIFY(plotCanvas != nullptr);

  plotCanvas->show();

  // add a variable to plot
  unsigned int var01 = plotCanvas->AddVariable("var01");
  QCOMPARE(plotCanvas->PlotCount(), 1u);
  QVERIFY(plotCanvas->PlotByVariable(var01) !=
      gazebo::gui::PlotCanvas::EmptyPlot);

  // find the curve associated with the variable
  gazebo::gui::PlotCurveWeakPtr curve = plotCanvas->PlotCurve(var01);
  auto c = curve.lock();
  QCOMPARE(c->Label(), std::string("var01"));

  // set new label and verify
  plotCanvas->SetVariableLabel(var01, "new_var01");
  QCOMPARE(c->Label(), std::string("new_var01"));

  plotCanvas->hide();
  delete plotCanvas;
}

// Generate a main function for the test
QTEST_MAIN(PlotCanvas_TEST)
