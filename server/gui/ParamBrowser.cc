#include "Param.hh"
#include "World.hh"
#include "Entity.hh"
#include "ParamBrowser.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Contructor
ParamBrowser::ParamBrowser(int x, int y, int w, int h, const char *l)
  : Fl_Scroll(x,y,w,h,l)
{
  this->end();
  this->resizable(NULL);
  this->lines = 0;
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
  this->lines = 0;
  this->redraw();
}

void ParamBrowser::AddDivider(const std::string& key, const std::string value)
{
  int y = this->y() + this->lines*20;
  MyOutput *out = new MyOutput(this->x(), y,this->w()*.5, 20);
  out->box(FL_BORDER_BOX);
  out->value(key.c_str());
  out->textcolor(fl_rgb_color(255,255,255));
  out->color(fl_rgb_color(100,100,100));

  MyOutput *out2 = new MyOutput(out->x() + out->w(), y, this->w()-out->w(),20);
  out2->box(FL_BORDER_BOX);
  out2->textcolor(fl_rgb_color(255,255,255));

  Entity *ent = World::Instance()->GetSelectedEntity();
  if (ent && ent->GetName() == value)
    out2->color(fl_rgb_color(3,20,179));
  else
    out2->color(fl_rgb_color(100,100,100));

  out2->value(value.c_str());
  out2->callback(ParamBrowser::DividerCB, this);

  this->lines++;

  this->add(out);
  this->add(out2);
  this->redraw();
}

void ParamBrowser::DividerCB(Fl_Widget *w, void *data)
{
  MyOutput *out = (MyOutput*)(w);
  std::string childName = out->value();

  out->color(fl_rgb_color(3,20,179), fl_rgb_color(255,255,255) );
  out->redraw();

  Entity *ent = World::Instance()->GetSelectedEntity();
  Entity *child = NULL;

  if (ent)
    child = ent->GetChild(childName);

  if (child)
    World::Instance()->SetSelectedEntity(child);

}

////////////////////////////////////////////////////////////////////////////////
/// Add a value to the param browser
void ParamBrowser::AddParam(Param *param)
{
  int y = this->y() + this->lines*20;
  MyOutput *out = new MyOutput(this->x(), y,this->w()*.5, 20);
  out->box(FL_BORDER_BOX);
  out->value(param->GetKey().c_str());

  MyInput *in = new MyInput(out->x() + out->w(), y, this->w()*.5,20);
  in->box(FL_BORDER_BOX);
  in->value(param->GetAsString().c_str());
  in->callback( &ParamBrowser::SetParam, this );
  in->when(FL_WHEN_ENTER_KEY | FL_WHEN_RELEASE );
  in->user_data( param );

  this->lines++;

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
