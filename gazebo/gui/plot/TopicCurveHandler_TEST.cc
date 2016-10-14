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

#include "gazebo/transport/transport.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/TopicCurveHandler.hh"
#include "gazebo/gui/plot/TopicCurveHandler_TEST.hh"

// common::Time g_sim_time;

/////////////////////////////////////////////////
void TopicCurveHandler_TEST::AddRemoveCurve()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // create a TopicCurveHandler and verify initial state
  gazebo::gui::TopicCurveHandler handler;
  QCOMPARE(handler.CurveCount(), 0u);

  // Create a new plot curve and add it to the handler
  gazebo::gui::PlotCurvePtr plotCurve01(new gazebo::gui::PlotCurve("curve01"));
  handler.AddCurve("/gazebo/default/world_stats?p=sim_time", plotCurve01);
  QCOMPARE(handler.CurveCount(), 1u);

  // verify we can have two different curves associated to the same topic
  gazebo::gui::PlotCurvePtr plotCurve02(new gazebo::gui::PlotCurve("curve02"));
  handler.AddCurve("/gazebo/default/world_stats?p=iterations", plotCurve02);
  QCOMPARE(handler.CurveCount(), 2u);

  // add another curve associated to a different topic
  gazebo::gui::PlotCurvePtr plotCurve03(new gazebo::gui::PlotCurve("curve03"));
  handler.AddCurve("/gazebo/default/pose/local/info?p=time",
      plotCurve03);
  QCOMPARE(handler.CurveCount(), 3u);

  // test removing curves
  handler.RemoveCurve(plotCurve01);
  QCOMPARE(handler.CurveCount(), 2u);
  handler.RemoveCurve(plotCurve02);
  QCOMPARE(handler.CurveCount(), 1u);
  handler.RemoveCurve(plotCurve03);
  QCOMPARE(handler.CurveCount(), 0u);
}

// Generate a main function for the test
QTEST_MAIN(TopicCurveHandler_TEST)
