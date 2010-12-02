#include <wx/sizer.h>
#include <wx/stattext.h>

#include "OgreAdaptor.hh"
#include "Scene.hh"
#include "FogPage.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
FogPage::FogPage(wxWindow *parent)
  : ParamPage(parent)
{
  wxString str;

  this->SetName(wxT("fog"));

  wxBoxSizer *boxSizer1 = new wxBoxSizer(wxVERTICAL);
  wxFlexGridSizer *gridSizer = new wxFlexGridSizer(5,2,2,2);

  gridSizer->Add(  
      new wxStaticText( this, wxID_ANY, wxT("Type:")),
      0,  wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL );

  this->typeBox = new wxComboBox(this, wxID_ANY, wxT("none"), wxDefaultPosition, wxDefaultSize, 0, NULL, wxCB_READONLY);
  this->typeBox->Insert(wxT("none"),0);
  this->typeBox->Insert(wxT("linear"),1);
  this->typeBox->Insert(wxT("exponential"),2);
  this->typeBox->Insert(wxT("exponential^2"),3);
  gridSizer->Add( this->typeBox, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL );


  gridSizer->Add(  
      new wxStaticText( this, wxID_ANY, wxT("Start:")),
      0,  wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL );

  this->startCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxTextValidator(wxFILTER_NUMERIC));

  str.Printf(wxT("%6.2f"), 1.0);
  this->startCtrl->SetValue(str);
  gridSizer->Add( this->startCtrl, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL );


  gridSizer->Add(  
      new wxStaticText( this, wxID_ANY, wxT("End:")),
      0,  wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL );

  this->endCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxTextValidator(wxFILTER_NUMERIC));
  str.Printf(wxT("%6.2f"), 100.0);
  this->endCtrl->SetValue(str);

  gridSizer->Add( this->endCtrl, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL );


  gridSizer->Add(  
      new wxStaticText( this, wxID_ANY, wxT("Density:")),
      0,  wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL );
  this->densityCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxTextValidator(wxFILTER_NUMERIC));

  str.Printf(wxT("%6.2f"), 1.0);
  this->densityCtrl->SetValue(str);
  gridSizer->Add( this->densityCtrl, 0, wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL );


  gridSizer->Add(  
      new wxStaticText( this, wxID_ANY, wxT("Color:")),
      0,  wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL );
  this->colorCtrl = new wxColourPickerCtrl(this, wxID_ANY, *wxWHITE);
//  this->colorCtrl->Connect(wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler(EditGridDialog::OnColorChange), NULL, this);
  gridSizer->Add(colorCtrl, 0, wxALIGN_CENTER_VERTICAL);


  boxSizer1->Add(gridSizer);

  this->SetSizer( boxSizer1 );
  this->Layout();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
FogPage::~FogPage()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Apply the current parameters
void FogPage::Apply()
{

  std::string str, type;
  str = std::string(this->typeBox->GetValue().mb_str());
  boost::trim(str);
  if (str == "exponential") 
    type = "exp";
  else if (str == "exponential^2")
    type = "exp2";
  else
    type = str;

  str = std::string(this->startCtrl->GetValue().mb_str());
  boost::trim(str);
  double start = boost::lexical_cast<double>(str);

  str = std::string(this->endCtrl->GetValue().mb_str());
  boost::trim(str);
  double end = boost::lexical_cast<double>(str);

  str = std::string(this->densityCtrl->GetValue().mb_str());
  boost::trim(str);
  double density = boost::lexical_cast<double>(str);

  wxColour clr = this->colorCtrl->GetColour();
  Color color(clr.Red(), clr.Green(), clr.Blue());

  OgreAdaptor::Instance()->GetScene(0)->SetFog( type, color, density, 
                                                start, end);
}
