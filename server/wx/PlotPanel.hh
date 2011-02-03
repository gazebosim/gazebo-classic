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

#ifndef PLOTPANEL_HH
#define PLOTPANEL_HH

#include <wx/wx.h>
#include <vector>

namespace gazebo
{
  class PlotPanel : public wxPanel
  {
    DECLARE_CLASS(PlotPanel)
    DECLARE_EVENT_TABLE()

    public: PlotPanel( wxWindow *parent  );
    public: virtual ~PlotPanel();

    public: void PushData(float x, float y);

    public: virtual void OnPaint(wxPaintEvent &evt);

    private: void Render(wxDC &dc);

    private: std::vector< std::pair<float, float> > points;
  };
}

#endif
