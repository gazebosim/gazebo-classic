#include "Param.hh"
#include "ParamBrowser.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Contructor
ParamBrowser::ParamBrowser(int x, int y, int w, int h, const char *l)
  : Fl_Scroll(x,y,w,h,l)
{
  this->end();
  this->resizable(NULL);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
ParamBrowser::~ParamBrowser()
{
}


////////////////////////////////////////////////////////////////////////////////
void ParamBrowser::Clear()
{
  this->clear();
  this->labels.clear();
  this->redraw();
}

void ParamBrowser::AddDivider(const std::string& key, const std::string value)
{
  int y = this->labels.size()*20;
  MyOutput *out = new MyOutput(this->x(), y,this->w()*.5, 20);
  out->box(FL_BORDER_BOX);
  out->value(key.c_str());
  out->textcolor(fl_rgb_color(255,255,255));
  out->color(fl_rgb_color(3,20,179),fl_rgb_color(255,255,255) );
  this->labels.push_back(out);

  MyOutput *out2 = new MyOutput(out->x() + out->w(), y, this->w()-out->w(),20);
  out2->box(FL_BORDER_BOX);
  out2->textcolor(fl_rgb_color(255,255,255));
  out2->color(fl_rgb_color(3,20,179), fl_rgb_color(255,255,255) );
  out2->value(value.c_str());

  this->add(out);
  this->add(out2);
  this->redraw();
}

////////////////////////////////////////////////////////////////////////////////
/// Add a value to the param browser
void ParamBrowser::AddParam(Param *param)
{
  int y = this->labels.size()*20;
  MyOutput *out = new MyOutput(this->x(), y,this->w()*.5, 20);
  out->box(FL_BORDER_BOX);
  out->value(param->GetKey().c_str());
  this->labels.push_back(out);

  MyInput *in = new MyInput(out->x() + out->w(), y, this->w()*.5,20);
  in->box(FL_BORDER_BOX);
  in->value(param->GetAsString().c_str());
  in->callback( &ParamBrowser::SetParam, this );
  in->when(FL_WHEN_ENTER_KEY | FL_WHEN_RELEASE );
  in->user_data( param );
  //this->inputs.push_back(in);

  this->add(out);
  this->add(in);
  this->redraw();
}

void ParamBrowser::SetParam(Fl_Widget *w, void *data)
{
  Fl_Input *in = (Fl_Input*)(w);
  Param *param = (Param*)(w->user_data());

  std::string value = in->value();
  param->SetFromString(value, true);
}
