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
#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/IncrementalPlot_TEST.hh"

/////////////////////////////////////////////////
void IncrementalPlot_TEST::AddRemoveCurve()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create a new plot
  gazebo::gui::IncrementalPlot *plot =
      new gazebo::gui::IncrementalPlot(nullptr);
  QVERIFY(plot != nullptr);

  // add a curve and verify that it is in the plot
  gazebo::gui::PlotCurveWeakPtr curve01 = plot->AddCurve("curve01");
  QVERIFY(!curve01.expired());
  auto c01 = curve01.lock();
  QVERIFY(c01 != nullptr);
  QVERIFY(plot->Curve(c01->Id()).lock() == c01);
  QVERIFY(plot->Curve(c01->Label()).lock() == c01);

  // add another curve and verify that both curves are in the plot
  gazebo::gui::PlotCurveWeakPtr curve02 = plot->AddCurve("curve02");
  QVERIFY(!curve02.expired());
  auto c02 = curve02.lock();
  QVERIFY(c02 != nullptr);
  QVERIFY(plot->Curve(c02->Id()).lock() == c02);
  QVERIFY(plot->Curve(c02->Label()).lock() == c02);

  // remove first curve
  plot->RemoveCurve(c01->Id());
  QVERIFY(plot->Curve(c01->Id()).expired());
  QVERIFY(plot->Curve(c01->Label()).expired());

  // remove second curve
  plot->RemoveCurve(c02->Id());
  QVERIFY(plot->Curve(c02->Id()).expired());
  QVERIFY(plot->Curve(c02->Label()).expired());

  // remove already removed curve
  plot->RemoveCurve(c02->Id());
  QVERIFY(plot->Curve(c02->Id()).expired());
  QVERIFY(plot->Curve(c02->Label()).expired());

  // check we can add more curves
  gazebo::gui::PlotCurveWeakPtr curve03 = plot->AddCurve("curve03");
  auto c03 = curve03.lock();

  QVERIFY(c03 != nullptr);
  QVERIFY(plot->Curve(c03->Id()).lock() == c03);
  QVERIFY(plot->Curve(c03->Label()).lock() == c03);

  // clear the plot - all curves should be removed
  plot->Clear();
  QVERIFY(plot->Curve(c03->Id()).expired());
  QVERIFY(plot->Curve(c03->Label()).expired());

  delete plot;
}

/////////////////////////////////////////////////
void IncrementalPlot_TEST::AttachDetachCurve()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create new plots
  gazebo::gui::IncrementalPlot *plot01 =
      new gazebo::gui::IncrementalPlot(nullptr);
  QVERIFY(plot01 != nullptr);

  gazebo::gui::IncrementalPlot *plot02 =
      new gazebo::gui::IncrementalPlot(nullptr);
  QVERIFY(plot02 != nullptr);

  // add a curve to plot01 and verify that it is in the right plot
  gazebo::gui::PlotCurveWeakPtr curve01 = plot01->AddCurve("curve01");
  QVERIFY(!curve01.expired());
  auto c01 = curve01.lock();
  QVERIFY(c01 != nullptr);
  QVERIFY(plot01->Curve(c01->Id()).lock() == c01);
  QVERIFY(plot02->Curve(c01->Id()).expired());

  // add another curve to plot01 and verify it is in the right plot
  gazebo::gui::PlotCurveWeakPtr curve02 = plot01->AddCurve("curve02");
  QVERIFY(!curve02.expired());
  auto c02 = curve02.lock();
  QVERIFY(c02 != nullptr);
  QVERIFY(plot01->Curve(c02->Id()).lock() == c02);
  QVERIFY(plot02->Curve(c02->Id()).expired());

  // detach curve01 from plot01 and verify it is no longer in the plot
  gazebo::gui::PlotCurvePtr pc01 = plot01->DetachCurve(c01->Id());
  QVERIFY(pc01 == c01);
  QVERIFY(plot01->Curve(c01->Id()).expired());
  QVERIFY(plot01->Curve(c01->Label()).expired());

  // attach curve01 to plot02 and verify
  plot02->AttachCurve(pc01);
  QVERIFY(plot02->Curve(pc01->Id()).lock() == c01);
  QVERIFY(plot02->Curve(pc01->Label()).lock() == c01);

  // detach curve02 from plot01
  gazebo::gui::PlotCurvePtr pc02 = plot01->DetachCurve(c02->Id());
  QVERIFY(pc02 == c02);
  QVERIFY(plot02->Curve(c02->Id()).expired());
  QVERIFY(plot02->Curve(c02->Label()).expired());

  // detach already datched curve02 from plot01
  gazebo::gui::PlotCurvePtr nullptrPc02 = plot01->DetachCurve(c02->Id());
  QVERIFY(nullptrPc02 == nullptr);

  // attach curve02 to plot02 and verify
  plot02->AttachCurve(pc02);
  QVERIFY(plot02->Curve(pc02->Id()).lock() == c02);
  QVERIFY(plot02->Curve(pc02->Label()).lock() == c02);

  // verify we can still add a curves to plot01 and plot02
  gazebo::gui::PlotCurveWeakPtr curve03 = plot01->AddCurve("curve03");
  QVERIFY(!curve03.expired());
  auto c03 = curve03.lock();
  QVERIFY(c03 != nullptr);
  QVERIFY(plot01->Curve(c03->Id()).lock() == c03);

  gazebo::gui::PlotCurveWeakPtr curve04 = plot02->AddCurve("curve04");
  QVERIFY(!curve04.expired());
  auto c04 = curve04.lock();
  QVERIFY(c04 != nullptr);
  QVERIFY(plot02->Curve(c04->Id()).lock() == c04);

  delete plot01;
  delete plot02;
}

/////////////////////////////////////////////////
void IncrementalPlot_TEST::AddPoint()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create new plots
  gazebo::gui::IncrementalPlot *plot =
      new gazebo::gui::IncrementalPlot(nullptr);
  QVERIFY(plot != nullptr);

  // add two curves
  gazebo::gui::PlotCurveWeakPtr curve01 = plot->AddCurve("curve01");
  QVERIFY(!curve01.expired());
  auto c01 = curve01.lock();
  QVERIFY(c01 != nullptr);
  QVERIFY(plot->Curve(c01->Id()).lock() == c01);

  gazebo::gui::PlotCurveWeakPtr curve02 = plot->AddCurve("curve02");
  QVERIFY(!curve02.expired());
  auto c02 = curve02.lock();
  QVERIFY(c02 != nullptr);
  QVERIFY(plot->Curve(c02->Id()).lock() == c02);

  // add point to curve01 and verify
  ignition::math::Vector2d point01(12.3, 99);
  QCOMPARE(c01->Size(), 0u);
  plot->AddPoint(c01->Id(), point01);
  QCOMPARE(c01->Size(), 1u);
  QCOMPARE(c01->Point(0u), point01);

  // add another point to curve01 and verify
  ignition::math::Vector2d point02(-1.3, -9.9);
  plot->AddPoint(c01->Id(), point02);
  QCOMPARE(c01->Size(), 2u);
  QCOMPARE(c01->Point(1u), point02);

  // add a list of points to curve02 and verify
  std::vector<ignition::math::Vector2d> points;
  unsigned int ptSize = 10;
  for (unsigned int i = 0; i < ptSize; ++i)
    points.push_back(ignition::math::Vector2d(i*2, i*0.5));
  plot->AddPoints(c02->Id(), points);
  QCOMPARE(c02->Size(), ptSize);
  for (unsigned int i = 0; i < ptSize; ++i)
    QCOMPARE(c02->Point(i), points[i]);

  delete plot;
}

/////////////////////////////////////////////////
void IncrementalPlot_TEST::SetCurveLabel()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create a new plot
  gazebo::gui::IncrementalPlot *plot =
      new gazebo::gui::IncrementalPlot(nullptr);
  QVERIFY(plot != nullptr);

  // add a curve and verify that it is in the plot
  gazebo::gui::PlotCurveWeakPtr curve01 = plot->AddCurve("curve01");
  QVERIFY(!curve01.expired());
  auto c01 = curve01.lock();
  QVERIFY(c01 != nullptr);
  QCOMPARE(c01->Label(), std::string("curve01"));
  QVERIFY(plot->Curve(c01->Label()).lock() == c01);

  plot->SetCurveLabel(c01->Id(), "new_curve01");
  QCOMPARE(c01->Label(), std::string("new_curve01"));
  QVERIFY(plot->Curve(c01->Label()).lock() == c01);

  delete plot;
}

// Generate a main function for the test
QTEST_MAIN(IncrementalPlot_TEST)
