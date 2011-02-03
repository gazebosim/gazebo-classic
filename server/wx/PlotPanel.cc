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

#include "PlotPanel.hh"

using namespace gazebo;

IMPLEMENT_CLASS(PlotPanel, wxPanel)
BEGIN_EVENT_TABLE(PlotPanel, wxPanel)
EVT_PAINT (PlotPanel::OnPaint)
END_EVENT_TABLE()

////////////////////////////////////////////////////////////////////////////////
PlotPanel::PlotPanel( wxWindow *parent  )
  : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(200,200), wxSUNKEN_BORDER) 
{
  this->SetBackgroundColour( *wxWHITE );
  this->SetDoubleBuffered(true);
}

////////////////////////////////////////////////////////////////////////////////
PlotPanel::~PlotPanel()
{
}

////////////////////////////////////////////////////////////////////////////////
void PlotPanel::PushData(float x, float y)
{
  this->points.push_back( std::make_pair<float, float>(x,y) );
}

////////////////////////////////////////////////////////////////////////////////
void PlotPanel::OnPaint(wxPaintEvent &evt)
{
  wxPaintDC dc(this);
  this->Render(dc);
}

////////////////////////////////////////////////////////////////////////////////
void PlotPanel::Render(wxDC &dc)
{
  wxSize size = dc.GetSize();
  //int xoffset = this->GetScrX();
  //int yoffset = this->GetScrY();

  int w = size.GetWidth();
  int h = size.GetHeight();

  dc.SetPen( wxPen( wxColor(0,0,0), 1 ) ); // black line, 3 pixels thick

  if (this->points.size() > 2)
  {
    float ymax = 0;
    for (unsigned int i=0; i < this->points.size(); i++)
    {
      if (this->points[i].second > ymax)
        ymax = this->points[i].second;
    }

    float yscale = h / ymax;
    float xscale = (float)w / this->points.size();
    printf("YMax[%f] Scale[%f]\n",ymax, yscale);
    printf("WH[%d %d]\n", w, h);

    for (unsigned int i=0; i < this->points.size()-1; i++)
    {
      int x1, y1;
      int x2, y2;

      x1 = i*xscale;
      y1 = this->points[i].second*yscale;

      x2 = (i+1)*xscale;
      y2 = this->points[i+1].second*yscale;
      printf("XY1[%d %d] XY2[%d %d]\n", x1, y1, x2, y2);

      dc.DrawLine( dc.DeviceToLogicalX(x1), dc.DeviceToLogicalY(y1), 
                   dc.DeviceToLogicalX(x2), dc.DeviceToLogicalY(y2));
    }
  }
}
