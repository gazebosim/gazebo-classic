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
#include <wx/stattext.h>
#include <wx/sizer.h>
#include <wx/valtext.h>
#include <wx/button.h>
#include <wx/colordlg.h>

#include "gui/GridPage.hh"

using namespace gazebo;
using namespace gui;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GridPage::GridPage(wxWindow *parent)
  : ParamPage(parent)//, wxID_ANY, wxDefaultPosition, wxSize(178,145))
{
  this->SetName(wxT("grid"));
  wxString str;

  // NATY: Fix this
  //Grid *grid = Simulator::Instance()->GetActiveWorld()->GetScene()->GetGrid(1);
  double size = grid->GetCellLength() * grid->GetCellCount();
  double spacing = size / grid->GetCellCount();

  wxFlexGridSizer *gridSizer = new wxFlexGridSizer(4,2,2,2);

  wxBoxSizer *boxSizer1 = new wxBoxSizer(wxVERTICAL);

  gridSizer->Add(  
      new wxStaticText( this, wxID_ANY, wxT("Grid Size:")),
      0,  wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL );

  this->gridSizeCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, 
                                       wxDefaultPosition, wxDefaultSize, 0, 
                                       wxTextValidator(wxFILTER_NUMERIC));
  str.Printf(wxT("%6.2f"), size);
  this->gridSizeCtrl->SetValue(str);
  gridSizer->Add( this->gridSizeCtrl, 0, 
                  wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL );

  gridSizer->Add(  
      new wxStaticText( this, wxID_ANY, wxT("Spacing:")),
      0, wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL );

  this->spacingCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxTextValidator(wxFILTER_NUMERIC));

  str.Printf(wxT("%6.2f"), spacing);
  this->spacingCtrl->SetValue(str);
  gridSizer->Add( this->spacingCtrl, 0, wxALIGN_CENTER_VERTICAL );

  gridSizer->Add(  
      new wxStaticText( this, wxID_ANY, wxT("Color:")),
      0, wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL );
  this->colorCtrl = new wxColourPickerCtrl(this, wxID_ANY, *wxWHITE);
  this->colorCtrl->Connect(wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler(GridPage::OnColorChange), NULL, this);
  gridSizer->Add(this->colorCtrl, 0, wxALIGN_CENTER_VERTICAL);

  boxSizer1->Add( gridSizer, 0, wxALL, 10 );


  this->SetSizer( boxSizer1 );
  this->Layout();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GridPage::~GridPage()
{
}

////////////////////////////////////////////////////////////////////////////////
// On cancel event
void GridPage::Apply()
{
  std::string str;
  // NATY: Fix this
  //Grid *grid = Simulator::Instance()->GetActiveWorld()->GetScene()->GetGrid(1);

  str = std::string(this->gridSizeCtrl->GetValue().mb_str());
  boost::trim(str);
  double size = boost::lexical_cast<double>(str);

  str = std::string(this->spacingCtrl->GetValue().mb_str());
  boost::trim(str);
  double spacing = boost::lexical_cast<double>(str);

  int count = size / spacing;

  wxColour clr = this->colorCtrl->GetColour();
  grid->SetColor( Color(clr.Red(), clr.Green(), clr.Blue()) );

  grid->SetCellLength( spacing );
  grid->SetCellCount( count );
}

////////////////////////////////////////////////////////////////////////////////
// On colour changed
void GridPage::OnColorChange(wxColourPickerEvent &event)
{
  wxColour color = event.GetColour();
  this->colorCtrl->SetColour(color);
}
