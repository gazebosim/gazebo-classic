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
#include "gazebo/gui/plot/PlotCurve_TEST.hh"

/////////////////////////////////////////////////
void PlotCurve_TEST::Curve()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create a new plot curve
  gazebo::gui::PlotCurve *plotCurve = new gazebo::gui::PlotCurve("curve01");

  QVERIFY(plotCurve != nullptr);

  // age
  QCOMPARE(plotCurve->Age(), 0u);
  plotCurve->SetAge(1u);
  QCOMPARE(plotCurve->Age(), 1u);

  // active
  QCOMPARE(plotCurve->Active(), true);
  plotCurve->SetActive(false);
  QCOMPARE(plotCurve->Active(), false);

  // label
  QCOMPARE(plotCurve->Label(), std::string("curve01"));
  plotCurve->SetLabel("new_curve01");
  QCOMPARE(plotCurve->Label(), std::string("new_curve01"));

  delete plotCurve;
}

/////////////////////////////////////////////////
void PlotCurve_TEST::CurveId()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // a set of unique plot curve ids
  std::set<unsigned int> ids;

  // Create new variable pills and verify they all have unique ids
  gazebo::gui::PlotCurve *curve01 = new gazebo::gui::PlotCurve("curve01");
  QVERIFY(curve01 != nullptr);
  unsigned int id = curve01->Id();
  QVERIFY(ids.count(id) == 0u);
  ids.insert(id);

  gazebo::gui::PlotCurve *curve02 = new gazebo::gui::PlotCurve("curve02");
  QVERIFY(curve02 != nullptr);
  id = curve02->Id();
  QVERIFY(ids.count(id) == 0u);
  ids.insert(id);

  gazebo::gui::PlotCurve *curve03 = new gazebo::gui::PlotCurve("curve03");
  QVERIFY(curve03 != nullptr);
  id = curve03->Id();
  QVERIFY(ids.count(id) == 0u);
  ids.insert(id);

  delete curve01;
  delete curve02;
  delete curve03;
}

/////////////////////////////////////////////////
void PlotCurve_TEST::AddPoint()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create a new plot curve
  gazebo::gui::PlotCurve *plotCurve = new gazebo::gui::PlotCurve("curve01");

  QVERIFY(plotCurve != nullptr);

  // verify the curve has no data
  QCOMPARE(plotCurve->Size(), 0u);

  // add points
  ignition::math::Vector2d point01(12.3, -39.4);
  plotCurve->AddPoint(point01);
  QCOMPARE(plotCurve->Size(), 1u);
  QCOMPARE(plotCurve->Point(0), point01);

  ignition::math::Vector2d point02(3.3, -3.4);
  plotCurve->AddPoint(point02);
  QCOMPARE(plotCurve->Size(), 2u);
  QCOMPARE(plotCurve->Point(1), point02);

  QCOMPARE(plotCurve->Min(), ignition::math::Vector2d(3.3, -39.4));
  QCOMPARE(plotCurve->Max(), ignition::math::Vector2d(12.3, -3.4));

  int tmpPointSize = plotCurve->Size();

  // create a list of points and add them to the curve
  std::vector<ignition::math::Vector2d> points;
  unsigned int ptSize = 100;
  for (unsigned int i = 0; i < ptSize; ++i)
    points.push_back(ignition::math::Vector2d(i, ptSize - i));

  plotCurve->AddPoints(points);

  // verify the points are appended to the curve
  QCOMPARE(plotCurve->Size(), tmpPointSize + ptSize);
  for (unsigned int i = 0; i < ptSize; ++i)
    QCOMPARE(plotCurve->Point(tmpPointSize + i), points[i]);

  delete plotCurve;
}

// Generate a main function for the test
QTEST_MAIN(PlotCurve_TEST)
