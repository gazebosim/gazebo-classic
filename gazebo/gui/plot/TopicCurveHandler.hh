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

#ifndef GAZEBO_GUI_PLOT_TOPICCURVEHANDLER_HH_
#define GAZEBO_GUI_PLOT_TOPICCURVEHANDLER_HH_

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

      /// \brief Add a curve to be updated
      /// \param[in] _query URI query string containing the param the curve is
      /// associated with.
      /// \param[in] _curve Pointer to the plot curve to add.
      public: void AddCurve(const std::string &_query, PlotCurveWeakPtr _curve);

      /// \brief Remove a curve from the topic data handler
      /// \param[in] _curve Pointer to the plot curve to remove.
      public: void RemoveCurve(PlotCurveWeakPtr _curve);

      /// \brief Get whether this topic data handler has the specified curve
      /// \param[in] _curve Pointer to the curve.
      /// \return True if curve exists
      public: bool HasCurve(PlotCurveWeakPtr _curve) const;

      /// \brief Get the number of curves managed by this handler
      /// \return Number of curves
      public: unsigned int CurveCount() const;

      /// \brief Private data pointer
      private: std::unique_ptr<TopicCurveHandlerPrivate> dataPtr;
    };
  }
}
#endif
