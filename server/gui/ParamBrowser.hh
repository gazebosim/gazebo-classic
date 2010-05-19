#ifndef PARAMBROWSER_HH
#define PARAMBROWSER_HH

#include <Fl/Fl_Scroll.H>
#include <Fl/Fl_Input.H>
#include <Fl/Fl_Output.H>
#include <Fl/fl_draw.H>
#include <vector>
#include <string>

namespace gazebo
{
  class Param;

  class ParamBrowser : public Fl_Scroll
  {
    /// \brief Contructor
    public: ParamBrowser(int x, int y, int w, int h, const char *l);

    /// \brief Destructor
    public: virtual ~ParamBrowser();

    public: void Clear();

    /// \brief Add a divider
    public: void AddDivider(const std::string& key, const std::string value);

    /// \brief Add a value to the param browser
    public: void AddParam(Param *param);

    public: static void SetParam(Fl_Widget *w, void *data);

    private: class MyOutput : public Fl_Output
             {
               public: MyOutput(int x, int y, int w, int h) : Fl_Output(x,y,w,h,"") {}
               public: void draw()
               {
                 if (input_type() == FL_HIDDEN_INPUT) 
                   return;

                 if (damage() & FL_DAMAGE_ALL)
                 {
                   draw_box(FL_FLAT_BOX,color());
                   fl_frame("XMMX",this->x(), this->y(),this->w(),this->h() );
                 }
                 Fl_Input_::drawtext( this->x()+2, this->y()+2,
                               w()-4, h()-4);
               }
             };

    private: class MyInput : public Fl_Input
             {
               public: MyInput(int x, int y, int w, int h) : Fl_Input(x,y,w,h,"") {}
               public: void draw()
               {
                 if (input_type() == FL_HIDDEN_INPUT) 
                   return;

                 if (damage() & FL_DAMAGE_ALL)
                 {
                   draw_box(FL_FLAT_BOX,color());
                   fl_frame("XMMX",this->x(), this->y(),this->w(),this->h() );
                 }
                 Fl_Input_::drawtext( this->x()+2, this->y()+2,
                               w()-4, h()-4);

               }
             };

    private: std::vector<MyOutput*> labels;
  };
}
#endif
