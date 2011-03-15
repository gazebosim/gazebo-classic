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
#include <deque>
#include <map>

#include "common/Color.hh"

namespace gazebo
{
	namespace gui
  {
    class PlotData
    {
      public: PlotData() {}
      public: virtual ~PlotData() {}
  
      public: common::Color color;
      public: std::deque<std::pair<float, float> > data;
    };
  
    class PlotPanel : public wxPanel
    {
      DECLARE_CLASS(PlotPanel)
      DECLARE_EVENT_TABLE()
  
      public: PlotPanel( wxWindow *parent  );
      public: virtual ~PlotPanel();
  
      public: void AddPlot(const std::string &timer);
  
      /// \brief Set the x-axis label
      public: void SetLabelX(const std::string lbl);
  
      /// \brief Set the dimensions of the xaxis
      public: void SetAxisX(float width);
  
      /// \brief Set the y-axis label
      public: void SetLabelY(const std::string lbl);
  
      public: void PushData(float x, float y, const std::string &timer);
  
      public: virtual void OnPaint(wxPaintEvent &evt);
  
      private: void Render(wxDC &dc);
  
      private: void Plot(wxDC &dc, PlotData *plot, float ymin, float ymax,
                         float xmin, float xmax, float xscale, float yscale, 
                         float yfactor);
  
      private: void GetBounds( PlotData *plot, float &ymin, float &ymax, float &xmin, float &xmax);
  
      // On mouse event callback
      private: void OnMouseEvent( wxMouseEvent &event);
  
      private: std::map<std::string, PlotData*> plots;
      private: int headIndex, tailIndex;
  
      private: wxString xLabel, yLabel;
      private: float xWidth;
      private: float w,h;
      private: float xmargin_left, xmargin_right;
      private: float ymargin_top, ymargin_bottom;
    };
  }
}
#endif
