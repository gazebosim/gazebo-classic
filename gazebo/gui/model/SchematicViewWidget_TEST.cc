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

#include "gazebo/gui/model/GraphView.hh"

#include "gazebo/gui/model/SchematicViewWidget.hh"
#include "gazebo/gui/model/SchematicViewWidget_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void SchematicViewWidget_TEST::AddRemove()
{
  gui::SchematicViewWidget *svWidget = new gui::SchematicViewWidget();
  QVERIFY(svWidget);

  // add nodes
  QCOMPARE(svWidget->GetNodeCount(), 0u);
  svWidget->AddNode("node_a");
  Q_ASSERT(svWidget->HasNode("node_a"));
  QCOMPARE(svWidget->GetNodeCount(), 1u);
  svWidget->AddNode("node_b");
  Q_ASSERT(svWidget->HasNode("node_b"));
  QCOMPARE(svWidget->GetNodeCount(), 2u);
  svWidget->AddNode("node_c");
  Q_ASSERT(svWidget->HasNode("node_c"));
  QCOMPARE(svWidget->GetNodeCount(), 3u);
  svWidget->AddNode("node_d");
  Q_ASSERT(svWidget->HasNode("node_d"));
  QCOMPARE(svWidget->GetNodeCount(), 4u);
  // remove node
  svWidget->RemoveNode("node_d");
  Q_ASSERT(!svWidget->HasNode("node_d"));
  QCOMPARE(svWidget->GetNodeCount(), 3u);
  // removing a node that doesn't exist doesn't break anything
  svWidget->RemoveNode("node_d");
  QCOMPARE(svWidget->GetNodeCount(), 3u);
  // add it back
  svWidget->AddNode("node_d");
  Q_ASSERT(svWidget->HasNode("node_d"));
  QCOMPARE(svWidget->GetNodeCount(), 4u);

  // add edges
  QCOMPARE(svWidget->GetEdgeCount(), 0u);
  svWidget->AddEdge("id_0", "edge_0", "revolute", "node_a", "node_b");
  QCOMPARE(svWidget->GetEdgeCount(), 1u);
  svWidget->AddEdge("id_1", "edge_1", "revolute2", "node_b", "node_c");
  QCOMPARE(svWidget->GetEdgeCount(), 2u);
  svWidget->AddEdge("id_2", "edge_2", "ball", "node_a", "node_c");
  QCOMPARE(svWidget->GetEdgeCount(), 3u);
  // remove edge
  svWidget->RemoveEdge("id_2");
  QCOMPARE(svWidget->GetEdgeCount(), 2u);
  // removing an edge that doesn't exist doesn't break anything
  svWidget->RemoveEdge("id_2");
  QCOMPARE(svWidget->GetEdgeCount(), 2u);
  // add it back
  svWidget->AddEdge("id_2", "edge_2", "prismatic", "node_a", "node_c");
  QCOMPARE(svWidget->GetEdgeCount(), 3u);

  // remove node and its edges
  svWidget->RemoveNode("node_b");
  QCOMPARE(svWidget->GetNodeCount(), 3u);
  QCOMPARE(svWidget->GetEdgeCount(), 1u);

  // update edge
  svWidget->UpdateEdge("id_2", "edge_2_update", "screw", "node_c", "node_a");
  QCOMPARE(svWidget->GetNodeCount(), 3u);
  QCOMPARE(svWidget->GetEdgeCount(), 1u);

  delete svWidget;
}

/////////////////////////////////////////////////
void SchematicViewWidget_TEST::Selection()
{
  gui::SchematicViewWidget *svWidget = new gui::SchematicViewWidget();
  QVERIFY(svWidget);

  QGraphicsView *view = svWidget->findChild<QGraphicsView *>();
  QVERIFY(view != NULL);

  svWidget->show();

  // add node a
  svWidget->AddNode("node_a");
  QList<QGraphicsItem *> items = view->scene()->items();
  QCOMPARE(items.size(), 1);
  QCOMPARE(view->scene()->selectedItems().size(), 0);

  // select node a
  QGraphicsItem *itemA = items[0];
  QVERIFY(itemA);
  QTest::mouseClick(view->viewport(), Qt::LeftButton, Qt::NoModifier,
      view->mapFromScene(itemA->mapToScene(1, 1)));
  QCoreApplication::processEvents();

  // verify selection
  QVERIFY(itemA->isSelected());
  QCOMPARE(view->scene()->selectedItems().size(), 1);

  // add node b
  svWidget->AddNode("node_b");
  items = view->scene()->items();
  QCOMPARE(items.size(), 2);
  QCOMPARE(view->scene()->selectedItems().size(), 1);

  // click to select node b and also make sure it deselects the previous node
  QGraphicsItem *itemB = items[0];
  QVERIFY(itemB);
  QTest::mouseClick(view->viewport(), Qt::LeftButton, Qt::NoModifier,
      view->mapFromScene(itemB->mapToScene(1, 1)));
  QCoreApplication::processEvents();

  // verify selection
  QVERIFY(itemB->isSelected());
  QVERIFY(!itemA->isSelected());
  QCOMPARE(view->scene()->selectedItems().size(), 1);

  // test multi select with control modifier. Select node a again
  QTest::mouseClick(view->viewport(), Qt::LeftButton, Qt::ControlModifier,
      view->mapFromScene(itemA->mapToScene(1, 1)));
  QCoreApplication::processEvents();

  QVERIFY(itemA->isSelected());
  QVERIFY(itemB->isSelected());
  QCOMPARE(view->scene()->selectedItems().size(), 2);

  // test deselect all - click outside of node
  QTest::mouseClick(view->viewport(), Qt::LeftButton, Qt::NoModifier,
      view->mapFromScene(itemA->mapToScene(-10, -10)));
  QCoreApplication::processEvents();

  QVERIFY(!itemA->isSelected());
  QVERIFY(!itemB->isSelected());
  QCOMPARE(view->scene()->selectedItems().size(), 0);

  // add edge 0
  svWidget->AddEdge("id_0", "edge_0", "prismatic", "node_a", "node_b");
  items = view->scene()->items();
  QCOMPARE(items.size(), 3);
  // items are returned in descending order and edges hav lower z value than
  // nodes so select the last item in the list
  QGraphicsItem *edge0 = items[2];
  QVERIFY(edge0);

  // select edge 0
  QTest::mouseClick(view->viewport(), Qt::LeftButton, Qt::NoModifier,
      view->mapFromScene(
      edge0->mapToScene(edge0->boundingRect().center().toPoint())));
  QCoreApplication::processEvents();

  // verify selection
  QVERIFY(!itemA->isSelected());
  QVERIFY(!itemB->isSelected());
  QVERIFY(edge0->isSelected());
  QCOMPARE(view->scene()->selectedItems().size(), 1);

  // test you can not select node and edge at the same time
  QTest::mouseClick(view->viewport(), Qt::LeftButton, Qt::ControlModifier,
      view->mapFromScene(itemA->mapToScene(1, 1)));
  QCoreApplication::processEvents();

  // only node a should be selected
  QVERIFY(itemA->isSelected());
  QVERIFY(!itemB->isSelected());
  QVERIFY(!edge0->isSelected());
  QCOMPARE(view->scene()->selectedItems().size(), 1);
}

// Generate a main function for the test
QTEST_MAIN(SchematicViewWidget_TEST)
