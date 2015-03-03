/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _MODELLISTWIDGET_TEST_HH_
#define _MODELLISTWIDGET_TEST_HH_

#include <string>
#include "gazebo/gui/QTestFixture.hh"

class QtProperty;
class QtTreePropertyBrowser;

/// \brief A test class for the ModelListWidget.
class ModelListWidget_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Response message received
  /// \brief _msg Message containing the response data
  private: void OnResponse(ConstResponsePtr &_msg);

  /// \brief Verify pose attributes and values.
  /// \param[in] _properties Pose properties.
  /// \param[in] _pose Expected pose values.
  private: void CheckPoseProperty(QList<QtProperty *> _properties,
      const gazebo::math::Pose &_pose);

  /// \brief Set pose values.
  /// \param[in] _propTreeBrowser Property browser.
  /// \param[in] _properties Pose properties.
  /// \param[in] _pose Expected pose values.
  private: void SetPoseProperty(QtTreePropertyBrowser *propTreeBrowser,
      QList<QtProperty *> _properties,
      const gazebo::math::Pose &_pose);

  /// \brief Test link property attributes and values.
  /// \param[in] _name Name of link.
  /// \param[in] _selfCollide True if links in model self collide.
  /// \param[in] _gravity True if gravity is enabled for this link.
  /// \param[in] _kinematic True if the link is in kinematic mode.
  /// \param[in] _canonical True if this is a canonical link.
  /// \param[in] _pose Expected pose values.
  private: void CheckLinkProperty(QList<QtProperty *> _properties,
    const std::string &_name, bool _selfCollide, bool _gravity, bool _kinematic,
    bool _canonical, const gazebo::math::Pose &_pose);

  /// \brief Set link property values.
  /// \param[in] _propTreeBrowser Property browser.
  /// \param[in] _properties Link properties.
  /// \param[in] _name Name of link.
  /// \param[in] _selfCollide New self collide value.
  /// \param[in] _gravity New gravity value.
  /// \param[in] _kinematic New kinematic value.
  /// \param[in] _canonical True if this is a canonical link and the pose
  /// should not be set.
  /// \param[in] _pose New pose values.
  private: void SetLinkProperty(QtTreePropertyBrowser *propTreeBrowser,
    QList<QtProperty *> _properties, const std::string &_name,
    bool _selfCollide, bool _gravity, bool _kinematic, bool _canonical,
    const gazebo::math::Pose &_pose);

  /// \brief Test to see the tree widget has correct items.
  private slots: void TreeWidget();

  /// \brief Test that the model widget item contains all models in the world.
  private slots: void ModelsTree();

  /// \brief Test that the property browser displays correct model properties.
  /// The test then modifies the properties, refresh the property browser, and
  /// verify the changes are set.
  private slots: void ModelProperties();

  /// \brief Test that the property browser displays correct link properties.
  /// This is similar to the ModelProperties test except the property browser
  /// now only displays link properties as the result of directly clicking on
  /// the link item in the models tree widget.
  private slots: void LinkProperties();
};

#endif
