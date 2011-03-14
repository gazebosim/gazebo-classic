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
