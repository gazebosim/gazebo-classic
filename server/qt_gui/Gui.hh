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
#ifndef GUI_HH
#define GUI_HH

#include "ui_sharon_simple.h"
#include "qt.h"

// void my_qt_setup(int argc, char **argv);
// void my_qt_run();

namespace gazebo
{

  class GLWindow;
  class XMLConfigNode;
  class Gui : public QMainWindow, private Ui::SharonWindow
  {
    Q_OBJECT

    public: Gui(int x, int y, int width, int height, const std::string &_name);
    public: virtual ~Gui();

    /// \brief Load the gui
    public: virtual void Load(XMLConfigNode *node);

    public: void Update();
    public: void Init();

    /// \brief Save the gui params in xml format
    public: virtual void Save(std::string &prefix, std::ostream &stream);


    public: void CreateCameras();

    protected: void closeEvent(QCloseEvent *_event);

    private: GLWindow *renderWidget;
  };
}

#endif
