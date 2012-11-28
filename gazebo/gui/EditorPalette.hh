/*
 * Copyright 2012 Nate Koenig
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
#ifndef _EDITOR_PALETTE_HH_
#define _EDITOR_PALETTE_HH_

#include "gui/qt.h"

#include "common/MouseEvent.hh"
#include "common/Event.hh"

namespace gazebo
{
  namespace gui
  {
    class EditorPalette : public QWidget
    {
      Q_OBJECT

      public: EditorPalette(QWidget *_parent = 0);
      public: virtual ~EditorPalette();
    };
  }
}

#endif
