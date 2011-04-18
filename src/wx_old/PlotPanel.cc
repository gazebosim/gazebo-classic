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

#include <boost/lexical_cast.hpp>
#include "gui/PlotPanel.hh"

using namespace gazebo;
using namespace gui;


IMPLEMENT_CLASS(PlotPanel, wxPanel)
BEGIN_EVENT_TABLE(PlotPanel, wxPanel)
EVT_PAINT (PlotPanel::OnPaint)
END_EVENT_TABLE()

int palette[][3] = {
  {0, 0, 0},
  {255, 0, 0},
  {0, 255, 0},
  {0, 0, 255},
  {255, 255, 0},
  {255, 0, 255},
  {0, 255, 255}
};


////////////////////////////////////////////////////////////////////////////////
PlotPanel::PlotPanel( wxWindow *parent  )
  : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(200,200), wxSUNKEN_BORDER) 
{
  this->SetBackgroundColour( *wxWHITE );
  this->SetDoubleBuffered(true);
  this->xLabel = wxT("undefined");
  this->yLabel = wxT("undefined");

  this->xWidth = 60.0;

  Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( PlotPanel::OnMouseEvent ), NULL, this );
}

////////////////////////////////////////////////////////////////////////////////
PlotPanel::~PlotPanel()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the x-axis label
void PlotPanel::SetLabelX(const std::string lbl)
{
  this->xLabel = wxString::FromAscii(lbl.c_str());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the y-axis label
void PlotPanel::SetLabelY(const std::string lbl)
{
  this->yLabel = wxString::FromAscii(lbl.c_str());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the dimensions of the xaxis
void PlotPanel::SetAxisX(float width)
{
  this->xWidth = width;
}

////////////////////////////////////////////////////////////////////////////////
void PlotPanel::AddPlot(const std::string &timer)
{
  std::map<std::string, PlotData*>::iterator iter;
  iter = this->plots.find(timer);
  if (iter != this->plots.end())
  {
    delete iter->second;
    this->plots.erase( iter );
  }
  else
  {
    int i = this->plots.size();
    this->plots[timer] = new PlotData();
    this->plots[timer]->color.Set(palette[i][0], palette[i][1], palette[i][2] );
  }
}

////////////////////////////////////////////////////////////////////////////////
void PlotPanel::PushData(float x, float y, const std::string &timer)
{
  std::map<std::string, PlotData*>::iterator iter;
  iter = this->plots.find(timer);

  if (iter != this->plots.end())
  {
    this->plots[timer]->data.push_back( std::make_pair<float, float>(x,y) );
    if (x - this->plots[timer]->data[0].first > this->xWidth+1)
      this->plots[timer]->data.pop_front();
  }
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
  if (this->plots.size() == 0)
    return;

  int textWidth, textHeight;
  wxSize size = dc.GetSize();

  this->w = size.GetWidth();
  this->h = size.GetHeight();

  this->xmargin_left = 50;
  this->xmargin_right = 20;

  this->ymargin_top = 20;
  this->ymargin_bottom = 30;

  dc.SetPen( wxPen( wxColor(200,200,200), 1 ) );

  // Draw the y axis
  dc.DrawLine( this->xmargin_left,this->h-this->ymargin_bottom, this->xmargin_left, this->ymargin_top );
  dc.DrawLine( w-this->xmargin_right, this->h-this->ymargin_bottom, w-this->xmargin_right, this->ymargin_top);

  // Draw the x axis
  dc.DrawLine(this->xmargin_left, this->h-this->ymargin_bottom, w-this->xmargin_right, this->h-this->ymargin_bottom);
  dc.DrawLine( this->xmargin_left, this->ymargin_top, w-this->xmargin_right, this->ymargin_top );

  int xLabelWidth, xLabelHeight;
  int yLabelWidth, yLabelHeight;

  dc.GetTextExtent(this->xLabel, &xLabelWidth, &xLabelHeight);
  dc.GetTextExtent(this->yLabel, &yLabelWidth, &yLabelHeight);

  dc.SetFont((wxFont&) *wxSMALL_FONT );

  // Draw the X axis label
  dc.DrawText( this->xLabel, (w-xLabelWidth-(this->xmargin_left+this->xmargin_right))*0.5 + this->xmargin_left, this->h-xLabelHeight/2-4);

  // Draw the Y axis label
  dc.DrawRotatedText( this->yLabel, 2, 
                      (this->h+yLabelWidth-this->ymargin_bottom+this->ymargin_top)*0.5, 90);

  float ymax, ymin = 0;
  float xmax, xmin = 0;

  std::map<std::string, PlotData*>::iterator iter;
  for (iter = this->plots.begin(); iter != this->plots.end(); iter++)
  {
    float y_min, y_max, x_min, x_max;
    this->GetBounds(iter->second, y_min, y_max, x_min, x_max);
    if (y_min < ymin) ymin = y_min;
    if (y_max > ymax) ymax = y_max;
    if (x_min < xmin) xmin = x_min;
    if (x_max > xmax) xmax = x_max;
  }

  int display_factor = 0;
  float yfactor = 1.0;
  while (ymax * yfactor < 0.1)
  {
    display_factor += 1;
    yfactor *= 10;
  }

  float xscale = (this->w-(this->xmargin_left+this->xmargin_right)) / this->xWidth;
  float yscale = (this->h-(this->ymargin_top + this->ymargin_bottom)) / (ymax*yfactor);

  // Draw the x-axis tic marks and labels
  dc.SetPen( wxPen( wxColor(200,200,200), 1, wxSHORT_DASH ) );
  for (float x=0; x <= this->xWidth; x+=5.0)
  {
    wxString str;
    str.Printf(wxT("%4.2f"), this->xWidth - x);

    dc.GetTextExtent(str, &textWidth, &textHeight);

    dc.DrawText( str, x*xscale + this->xmargin_left - textWidth*0.5, this->h-28);
    dc.DrawLine(x*xscale+this->xmargin_left, this->ymargin_top, 
        x*xscale+this->xmargin_left, this->h-this->ymargin_bottom );
  }

  float yt =  (ymax-ymin) / 4.0;
  // Draw the y-axis tic marks and labels
  dc.SetPen( wxPen( wxColor(200,200,200), 1, wxSHORT_DASH ) );
  for (float y=ymin; y <= ymax; y+= yt)
  {
    int ypos = (int)rint(this->h - y*yscale*yfactor - this->ymargin_bottom);

    wxString str;
    str.Printf(wxT("%4.2f"), y*yfactor);

    dc.GetTextExtent(str, &textWidth, &textHeight);

    dc.DrawText( str, 20, ypos - textHeight/2.0);
    dc.DrawLine(this->xmargin_left, ypos, this->w-this->xmargin_right, ypos );
  }

  wxString yfactorbase = wxT("x10");
  dc.GetTextExtent(yfactorbase, &textWidth, &textHeight);

  dc.DrawText( wxT("x10"), this->xmargin_left, 8);
  std::string tmp_str = "-";
  tmp_str += boost::lexical_cast<std::string>(display_factor);
  dc.DrawText( wxString::FromAscii(tmp_str.c_str()), this->xmargin_left+textWidth, 2);

  int i=0;
  for (iter = this->plots.begin(); iter != this->plots.end(); iter++, i++)
  {
    dc.SetTextForeground( wxColor(iter->second->color.R()*255,
                                  iter->second->color.G()*255,
                                  iter->second->color.B()*255) );

    std::string str = iter->first;
    dc.DrawText( wxString::FromAscii( str.c_str() ), 
        this->xmargin_left+15, this->ymargin_top+5 + 15*i);

    this->Plot( dc, iter->second, ymin, ymax, xmin, xmax,
       xscale, yscale, yfactor );
  }

}

void PlotPanel::GetBounds( PlotData *plot, float &ymin, float &ymax, float &xmin, float &xmax)
{
  xmax = plot->data[0].first;
  xmin = plot->data[0].first;

  ymax = plot->data[0].second;
  ymin = plot->data[0].second;

  for (unsigned int i=1; i < plot->data.size(); i++)
  {
    if (plot->data[i].second > ymax)
      ymax = plot->data[i].second;
    if (plot->data[i].second < ymin)
      ymin = plot->data[i].second;

    if (plot->data[i].first > xmax)
      xmax = plot->data[i].first;
    if (plot->data[i].first < xmin)
      xmin = plot->data[i].first;
  }
  //ymax += (ymax-ymin) * 0.25;
}

void PlotPanel::Plot(wxDC &dc, PlotData *plot, float ymin, float ymax,
    float xmin, float xmax, float xscale, float yscale, float yfactor)
{
  if (plot->data.size() > 2)
  {
    dc.SetPen( wxPen( wxColor(plot->color.R()*255,
                              plot->color.G()*255,
                              plot->color.B()*255), 1 ) );
    for (unsigned int i=plot->data.size()-1; i > 0; i--)
    {
      float x1, y1;
      float x2, y2;

      x1 = this->xWidth - xmax + plot->data[i].first;
      x1 = rint(x1*xscale + this->xmargin_left);
      y1 = rint(this->h - plot->data[i].second*yscale*yfactor - this->ymargin_bottom);

      x2 = this->xWidth - xmax + plot->data[i-1].first;
      x2 = rint(x2*xscale + this->xmargin_left);
      y2 = rint(this->h - plot->data[i-1].second*yscale*yfactor - this->ymargin_bottom);

      dc.DrawLine( x1, y1, x2, y2);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// On mouse event callback
void PlotPanel::OnMouseEvent( wxMouseEvent &event)
{
  /*this->mouseEvent.pos.Set( event.GetX(), event.GetY() );

  if (event.LeftDown() || event.MiddleDown() || event.RightDown())
    this->mouseEvent.pressPos = this->mouseEvent.pos;

  this->mouseEvent.left = event.LeftIsDown() ? MouseEvent::DOWN : MouseEvent::UP;
  this->mouseEvent.right = event.RightIsDown() ? MouseEvent::DOWN : MouseEvent::UP;
  this->mouseEvent.middle = event.MiddleIsDown() ? MouseEvent::DOWN : MouseEvent::UP;
  */

  if (event.GetWheelRotation() < 0) 
  {
    this->xWidth += 1.0;
    //this->mouseEvent.scroll.y = 1;
    //this->mouseEvent.middle = MouseEvent::SCROLL;
  }
  else if (event.GetWheelRotation() > 0)
  {
    this->xWidth -= 1.0;
    //this->mouseEvent.scroll.y = -1;
    //this->mouseEvent.middle = MouseEvent::SCROLL;
  }

  //this->mouseEvent.dragging = event.Dragging();
}
