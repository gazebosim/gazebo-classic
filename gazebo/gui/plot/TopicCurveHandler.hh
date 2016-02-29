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

#ifndef _GAZEBO_GUI_PLOT_TOPICCURVEHANDLER_HH_
#define _GAZEBO_GUI_PLOT_TOPICCURVEHANDLER_HH_

#include <memory>
#include <string>

#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/util/system.hh"

namespace google
{
  namespace protobuf
  {
    class Message;
  }
}

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class TopicCurveHandlerPrivate;

    /// \brief Manages and updates curves based on topic data.
    class GZ_GUI_VISIBLE TopicCurveHandler
    {
      /// \brief Constructor.
      public: TopicCurveHandler();

      /// \brief Destructor.
      public: ~TopicCurveHandler();

      /// \brief Set topic to subscribe to.
      /// \param[in] _topic Name of topic
      public: void SetTopic(const std::string &_topic);

      /// \brief Add a curve to be updated
      /// \param[in] _query URI query string containing the param the curve is
      /// associated with.
      /// \param[in] _curve Pointer to the plot curve to add.
      public: void AddCurve(const std::string &_query, PlotCurveWeakPtr _curve);

      /// \brief Remove a curve from the topic data hander
      /// \param[in] _curve Pointer to the plot curve to remove.
      public: void RemoveCurve(PlotCurveWeakPtr _curve);

      /// \brief Get whether this topic data handler has the specified curve
      /// \return True if curve exists
      public: bool HasCurve(PlotCurveWeakPtr _curve) const;

      /// \brief Get the number of curves managed by this handler
      /// \return Number of curves
      public: unsigned int CurveCount() const;

      /// \brief Topic data callback
      /// \param[in] _msg Message data
      public: void OnTopicData(const std::string &_msg);

      /// \brief Update th plot curve based on message
      /// \param[in] _msg Message containing data to be added to the curve
      /// \param[in] _index Index of token in the param path string
      public: void UpdateCurve(google::protobuf::Message *_msg,
                  const int _index = 0);

      /// \brief Private data pointer
      private: std::unique_ptr<TopicCurveHandlerPrivate> dataPtr;
    };
  }
}
#endif
