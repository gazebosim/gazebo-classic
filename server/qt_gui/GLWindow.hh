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
#ifndef GL_WIDGET_HH
#define GL_WIDGET_HH

#include <string>
#include <vector>

#include "qt.h"

namespace gazebo
{
  class UserCamera;
  class GLWindow : public QWidget
  {
    Q_OBJECT

    public: GLWindow(QWidget *_parent = 0);
    public: virtual ~GLWindow();

    //public: UserCamera GetCamera() const;

    public: void Init();
    public: void CreateCameras();
    public: void Clear();

    public slots: void AngledView();
    public slots: void SideView();
    public slots: void FrontView();

    private: void showEvent(QShowEvent *_event);
    private: void paintEvent(QPaintEvent *_e);
    private: void resizeEvent(QResizeEvent *_e);
    private: bool eventFilter(QObject *_obj, QEvent *_event);
    protected: void keyReleaseEvent(QKeyEvent *_event);

    public: std::string GetOgreHandle() const;

    private: int windowId;

    private: UserCamera *userCamera;
    private: QFrame *renderFrame;
  };
}

#endif
