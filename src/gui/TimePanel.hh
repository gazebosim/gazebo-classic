/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef TIME_PANEL_HH
#define TIME_PANEL_HH

#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include "common/Time.hh"

namespace gazebo
{
	namespace gui
  {
    class TimePanel : public wxPanel
    {
      public: TimePanel( wxWindow *parent );
      public: virtual ~TimePanel();
  
      public: void MyUpdate();
  
      private: wxStaticText *percentRealTimeText;
      private: wxTextCtrl *percentRealTimeCtrl;
  
      private: wxStaticText *simTimeText;
      private: wxTextCtrl *simTimeCtrl;
  
      private: wxStaticText *realTimeText;
      private: wxTextCtrl *realTimeCtrl;
  
      private: wxStaticText *pauseTimeText;
      private: wxTextCtrl *pauseTimeCtrl;
  
      private: common::Time lastUpdateTime,statusUpdatePeriod;
      private: common::Time percentLastRealTime, percentLastSimTime;
    };
  }
}
#endif
