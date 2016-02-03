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

#ifndef _GAZEBO_GUI_PLOT_PLOTPALETTE_PRIVATE_HH_
#define _GAZEBO_GUI_PLOT_PLOTPALETTE_PRIVATE_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/transport/TransportIface.hh"

namespace gazebo
{
  namespace gui
  {
    class ConfigWidget;

    /// \brief Private data for the PlotPalette class
    class PlotPalettePrivate
    {
      public: ConfigWidget *topicsTop;
      public: ConfigWidget *modelsTop;

      /// \brief The list of diagnostic labels.
      public: ConfigWidget *topicsBottom;
      public: ConfigWidget *modelsBottom;
      public: ConfigWidget *simBottom;
      public: ConfigWidget *searchBottom;
      public: ConfigWidget *searchArea;

      /// \brief Transport node used for communication.
      public: transport::NodePtr node;

      /// \brief Subscribe to model info messages.
      public: transport::SubscriberPtr newModelSub;

      /// \brief Publish request messages.
      public: transport::PublisherPtr requestPub;
      public: transport::SubscriberPtr requestSub;

      /// \brief Subscribe to response messages.
      public: transport::SubscriberPtr responseSub;

      /// \brief Message used to field requests.
      public: msgs::Request *requestMsg;



      public: QStandardItemModel *searchModel;
    };

    /// \brief Private data for the ItemConfigWidget class
    class ItemConfigWidgetPrivate
    {
      public: std::string text;
      public: std::string plotInfo;
      public: bool draggable;
    };
  }
}
#endif
