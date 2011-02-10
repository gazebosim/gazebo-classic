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

#ifndef DIAGNOSTICSDIALOG_HH
#define DIAGNOSTICSDIALOG_HH

#include <wx/wx.h>
#include <wx/treectrl.h>
#include <wx/textctrl.h>

#include <map>
#include <vector>

#include "Time.hh"

namespace gazebo
{
  class PlotPanel;

  class DiagnosticsDialog : public wxDialog
  {
    public: DiagnosticsDialog( wxWindow *parent );
    public: virtual ~DiagnosticsDialog();

    public: void Update();

    private: void TimerStopCB(std::string timer);

    private: void OnTreeClick(wxTreeEvent &event);

    private: wxTreeCtrl *treeCtrl;

    private: wxTextCtrl *timerNameCtrl;
    private: wxTextCtrl *timerElapsedCtrl;
    private: PlotPanel *plot;

    private: std::map< std::string, std::vector< std::pair<Time,Time> > > times;
  };
}

#endif
