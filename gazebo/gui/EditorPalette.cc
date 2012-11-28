/*
 * Copyright 2011 Nate Koenig
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

#include "gui/EditorPalette.hh"

using namespace gazebo;
using namespace gui;


EditorPalette::EditorPalette(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("editorPalette");

  QHBoxLayout *hboxLayout = new QHBoxLayout;

  QLabel *modelLabel = new QLabel(tr("Model: "));
  hboxLayout->addWidget(modelLabel);

  QVBoxLayout *mainLayout = new QVBoxLayout;

/*  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->addWidget(label, 0);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);
  mainLayout->addWidget(frame);*/

  mainLayout->addLayout(hboxLayout);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);
}

EditorPalette::~EditorPalette()
{

}
