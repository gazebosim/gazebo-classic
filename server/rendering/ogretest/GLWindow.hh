#ifndef GLWINDOW_HH
#define GLWINDOW_HH

#include <FL/Fl_Gl_Window.H>

class GLWindow : public Fl_Gl_Window
{
  public: GLWindow(int x, int y, int w, int h, const char *t=NULL );
  public: virtual ~GLWindow();
  public: virtual void Init();

  public: void Update();

  public: void resize(int x, int y, int w, int h);
};

#endif
