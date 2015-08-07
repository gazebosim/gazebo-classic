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

#include <gtest/gtest.h>

#include "test_config.h"
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/gazebo_config.h"
#include "test/util.hh"

using namespace gazebo;

class MouseEvent : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(MouseEvent, CopyConstructor)
{
  ignition::math::Vector2i pos(1.5, 25);
  ignition::math::Vector2i prevPos(1.6, 26);
  ignition::math::Vector2i pressPos(2.5, 25);
  ignition::math::Vector2i scroll(2, 3);
  float moveScale = 0.3;
  bool dragging = true;
  common::MouseEvent::EventType type = common::MouseEvent::PRESS;
  common::MouseEvent::MouseButton button = common::MouseEvent::LEFT;
  bool shift = false;
  bool alt = false;
  bool control = false;

  // create mouse event
  common::MouseEvent event;
  event.SetPos(pos);
  event.SetPrevPos(prevPos);
  event.SetPressPos(pressPos);
  event.SetScroll(scroll);
  event.SetMoveScale(moveScale);
  event.SetDragging(dragging);
  event.SetType(type);
  event.SetButton(button);
  event.SetShift(shift);
  event.SetAlt(alt);
  event.SetControl(control);

  // verify mouse event values
  EXPECT_EQ(event.Pos(), pos);
  EXPECT_EQ(event.PrevPos(), prevPos);
  EXPECT_EQ(event.PressPos(), pressPos);
  EXPECT_EQ(event.Scroll(), scroll);
  EXPECT_DOUBLE_EQ(event.MoveScale(), moveScale);
  EXPECT_EQ(event.Dragging(), dragging);
  EXPECT_EQ(event.Type(), type);
  EXPECT_EQ(event.Button(), button);
  EXPECT_EQ(event.Shift(), shift);
  EXPECT_EQ(event.Alt(), alt);
  EXPECT_EQ(event.Control(), control);

  // verify copy constructor works
  common::MouseEvent otherEvent = event;
  EXPECT_EQ(otherEvent.Pos(), pos);
  EXPECT_EQ(otherEvent.PrevPos(), prevPos);
  EXPECT_EQ(otherEvent.PressPos(), pressPos);
  EXPECT_EQ(otherEvent.Scroll(), scroll);
  EXPECT_DOUBLE_EQ(otherEvent.MoveScale(), moveScale);
  EXPECT_EQ(otherEvent.Dragging(), dragging);
  EXPECT_EQ(otherEvent.Type(), type);
  EXPECT_EQ(otherEvent.Button(), button);
  EXPECT_EQ(otherEvent.Shift(), shift);
  EXPECT_EQ(otherEvent.Alt(), alt);
  EXPECT_EQ(otherEvent.Control(), control);

  common::MouseEvent otherEvent2(event);
  EXPECT_EQ(otherEvent2.Pos(), pos);
  EXPECT_EQ(otherEvent2.PrevPos(), prevPos);
  EXPECT_EQ(otherEvent2.PressPos(), pressPos);
  EXPECT_EQ(otherEvent2.Scroll(), scroll);
  EXPECT_DOUBLE_EQ(otherEvent2.MoveScale(), moveScale);
  EXPECT_EQ(otherEvent2.Dragging(), dragging);
  EXPECT_EQ(otherEvent2.Type(), type);
  EXPECT_EQ(otherEvent2.Button(), button);
  EXPECT_EQ(otherEvent2.Shift(), shift);
  EXPECT_EQ(otherEvent2.Alt(), alt);
  EXPECT_EQ(otherEvent2.Control(), control);

  // update the original mouse event values
  ignition::math::Vector2i newPos(3.4, 18);
  ignition::math::Vector2i newPrevPos(3.2, 17);
  ignition::math::Vector2i newPressPos(3.0, 16);
  ignition::math::Vector2i newScroll(1, 2);
  float newMoveScale = 0.4;
  bool newDragging = false;
  common::MouseEvent::EventType newType = common::MouseEvent::RELEASE;
  common::MouseEvent::MouseButton newButton = common::MouseEvent::LEFT;
  bool newShift = true;
  bool newAlt = false;
  bool newControl = false;

  event.SetPos(newPos);
  event.SetPrevPos(newPrevPos);
  event.SetPressPos(newPressPos);
  event.SetScroll(newScroll);
  event.SetMoveScale(newMoveScale);
  event.SetDragging(newDragging);
  event.SetType(newType);
  event.SetButton(newButton);
  event.SetShift(newShift);
  event.SetAlt(newAlt);
  event.SetControl(newControl);

  // verify new mouse event values
  EXPECT_EQ(event.Pos(), newPos);
  EXPECT_EQ(event.PrevPos(), newPrevPos);
  EXPECT_EQ(event.PressPos(), newPressPos);
  EXPECT_EQ(event.Scroll(), newScroll);
  EXPECT_DOUBLE_EQ(event.MoveScale(), newMoveScale);
  EXPECT_EQ(event.Dragging(), newDragging);
  EXPECT_EQ(event.Type(), newType);
  EXPECT_EQ(event.Button(), newButton);
  EXPECT_EQ(event.Shift(), newShift);
  EXPECT_EQ(event.Alt(), newAlt);
  EXPECT_EQ(event.Control(), newControl);

  // verify copies have the original values
  EXPECT_EQ(otherEvent.Pos(), pos);
  EXPECT_EQ(otherEvent.PrevPos(), prevPos);
  EXPECT_EQ(otherEvent.PressPos(), pressPos);
  EXPECT_EQ(otherEvent.Scroll(), scroll);
  EXPECT_DOUBLE_EQ(otherEvent.MoveScale(), moveScale);
  EXPECT_EQ(otherEvent.Dragging(), dragging);
  EXPECT_EQ(otherEvent.Type(), type);
  EXPECT_EQ(otherEvent.Button(), button);
  EXPECT_EQ(otherEvent.Shift(), shift);
  EXPECT_EQ(otherEvent.Alt(), alt);
  EXPECT_EQ(otherEvent.Control(), control);

  EXPECT_EQ(otherEvent2.Pos(), pos);
  EXPECT_EQ(otherEvent2.PrevPos(), prevPos);
  EXPECT_EQ(otherEvent2.PressPos(), pressPos);
  EXPECT_EQ(otherEvent2.Scroll(), scroll);
  EXPECT_DOUBLE_EQ(otherEvent2.MoveScale(), moveScale);
  EXPECT_EQ(otherEvent2.Dragging(), dragging);
  EXPECT_EQ(otherEvent2.Type(), type);
  EXPECT_EQ(otherEvent2.Button(), button);
  EXPECT_EQ(otherEvent2.Shift(), shift);
  EXPECT_EQ(otherEvent2.Alt(), alt);
  EXPECT_EQ(otherEvent2.Control(), control);
}

/////////////////////////////////////////////////
TEST_F(MouseEvent, Assignment)
{
  ignition::math::Vector2i pos(1.5, 25);
  ignition::math::Vector2i prevPos(1.6, 26);
  ignition::math::Vector2i pressPos(2.5, 25);
  ignition::math::Vector2i scroll(2, 3);
  float moveScale = 0.3;
  bool dragging = true;
  common::MouseEvent::EventType type = common::MouseEvent::PRESS;
  common::MouseEvent::MouseButton button = common::MouseEvent::LEFT;
  bool shift = false;
  bool alt = false;
  bool control = false;

  // create mouse event
  common::MouseEvent event;
  event.SetPos(pos);
  event.SetPrevPos(prevPos);
  event.SetPressPos(pressPos);
  event.SetScroll(scroll);
  event.SetMoveScale(moveScale);
  event.SetDragging(dragging);
  event.SetType(type);
  event.SetButton(button);
  event.SetShift(shift);
  event.SetAlt(alt);
  event.SetControl(control);

  // verify mouse event values
  EXPECT_EQ(event.Pos(), pos);
  EXPECT_EQ(event.PrevPos(), prevPos);
  EXPECT_EQ(event.Scroll(), scroll);
  EXPECT_DOUBLE_EQ(event.MoveScale(), moveScale);
  EXPECT_EQ(event.Dragging(), dragging);
  EXPECT_EQ(event.Type(), type);
  EXPECT_EQ(event.Button(), button);
  EXPECT_EQ(event.Shift(), shift);
  EXPECT_EQ(event.Alt(), alt);
  EXPECT_EQ(event.Control(), control);

  // verify asignment operator works
  common::MouseEvent otherEvent;
  otherEvent = event;
  EXPECT_EQ(otherEvent.Pos(), pos);
  EXPECT_EQ(otherEvent.PrevPos(), prevPos);
  EXPECT_EQ(otherEvent.PressPos(), pressPos);
  EXPECT_EQ(otherEvent.Scroll(), scroll);
  EXPECT_DOUBLE_EQ(otherEvent.MoveScale(), moveScale);
  EXPECT_EQ(otherEvent.Dragging(), dragging);
  EXPECT_EQ(otherEvent.Type(), type);
  EXPECT_EQ(otherEvent.Button(), button);
  EXPECT_EQ(otherEvent.Shift(), shift);
  EXPECT_EQ(otherEvent.Alt(), alt);
  EXPECT_EQ(otherEvent.Control(), control);

  // update the original mouse event values
  ignition::math::Vector2i newPos(3.4, 18);
  ignition::math::Vector2i newPrevPos(3.2, 17);
  ignition::math::Vector2i newPressPos(3.0, 16);
  ignition::math::Vector2i newScroll(1, 2);
  float newMoveScale = 0.4;
  bool newDragging = false;
  common::MouseEvent::EventType newType = common::MouseEvent::RELEASE;
  common::MouseEvent::MouseButton newButton = common::MouseEvent::LEFT;
  bool newShift = false;
  bool newAlt = true;
  bool newControl = false;

  event.SetPos(newPos);
  event.SetPrevPos(newPrevPos);
  event.SetPressPos(newPressPos);
  event.SetScroll(newScroll);
  event.SetMoveScale(newMoveScale);
  event.SetDragging(newDragging);
  event.SetType(newType);
  event.SetButton(newButton);
  event.SetShift(newShift);
  event.SetAlt(newAlt);
  event.SetControl(newControl);

  // verify new mouse event values
  EXPECT_EQ(event.Pos(), newPos);
  EXPECT_EQ(event.PrevPos(), newPrevPos);
  EXPECT_EQ(event.PressPos(), newPressPos);
  EXPECT_EQ(event.Scroll(), newScroll);
  EXPECT_DOUBLE_EQ(event.MoveScale(), newMoveScale);
  EXPECT_EQ(event.Dragging(), newDragging);
  EXPECT_EQ(event.Type(), newType);
  EXPECT_EQ(event.Button(), newButton);
  EXPECT_EQ(event.Shift(), newShift);
  EXPECT_EQ(event.Alt(), newAlt);
  EXPECT_EQ(event.Control(), newControl);

  // verify the copy has the original values
  EXPECT_EQ(otherEvent.Pos(), pos);
  EXPECT_EQ(otherEvent.PrevPos(), prevPos);
  EXPECT_EQ(otherEvent.PressPos(), pressPos);
  EXPECT_EQ(otherEvent.Scroll(), scroll);
  EXPECT_DOUBLE_EQ(otherEvent.MoveScale(), moveScale);
  EXPECT_EQ(otherEvent.Dragging(), dragging);
  EXPECT_EQ(otherEvent.Type(), type);
  EXPECT_EQ(otherEvent.Button(), button);
  EXPECT_EQ(otherEvent.Shift(), shift);
  EXPECT_EQ(otherEvent.Alt(), alt);
  EXPECT_EQ(otherEvent.Control(), control);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
